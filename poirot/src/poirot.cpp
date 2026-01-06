// Copyright 2025 Miguel Ángel González Santamarta
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <curl/curl.h>
#include <dirent.h>
#include <syscall.h>

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <random>
#include <set>

#include <poirot_msgs/msg/function_call.hpp>
#include <poirot_msgs/msg/function_stats.hpp>
#include <poirot_msgs/msg/profiling_data.hpp>

#include "poirot/poirot.hpp"

using namespace poirot;

// ============================================================================
// Constants
// ============================================================================
static constexpr double DEFAULT_CO2_FACTOR_KG_PER_KWH = 0.475; // World average
static constexpr double MIN_TDP_WATTS = 15.0;
static constexpr double MAX_TDP_WATTS = 400.0;
static constexpr double IDLE_POWER_FACTOR =
    0.15; // CPUs consume ~15% TDP at idle
static constexpr long CURL_TIMEOUT_SECONDS = 10L;

// ============================================================================
// CURL callback for downloading CO2 factors
// ============================================================================
static size_t WriteCallback(void *contents, size_t size, size_t nmemb,
                            void *userp) {
  static_cast<std::string *>(userp)->append(static_cast<char *>(contents),
                                            size * nmemb);
  return size * nmemb;
}

// ============================================================================
// Poirot Implementation
// ============================================================================
/**
 * @brief Generates a unique UUID as a string.
 *
 * This function uses random numbers to generate a 16-character hexadecimal
 * UUID.
 *
 * @return A string containing a 16-character hexadecimal UUID.
 */
inline std::string generateUUID() {
  static constexpr char hex_digits[] = "0123456789abcdef";
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> dis(0, 15);

  std::string result;
  result.reserve(16);
  for (int i = 0; i < 16; ++i) {
    result += hex_digits[dis(gen)];
  }
  return result;
}

Poirot::Poirot() {
  this->node_ = rclcpp::Node::make_shared("yasmin_" + generateUUID() + "_node");
  this->profiling_data_publisher_ =
      this->node_->create_publisher<poirot_msgs::msg::ProfilingData>(
          "poirot/data", rclcpp::QoS(10));
  this->auto_configure();
}

Poirot::~Poirot() { this->print_summary(); }

void Poirot::auto_configure() {
  this->download_co2_factors();
  this->detect_system_info();
  this->search_hwmon_paths();

  std::lock_guard<std::mutex> lock(this->cpu_read_mutex_);
  this->prev_cpu_read_time_ = std::chrono::high_resolution_clock::now();
}

bool Poirot::download_co2_factors() {
  static std::once_flag curl_init_flag;
  std::call_once(curl_init_flag,
                 []() { curl_global_init(CURL_GLOBAL_DEFAULT); });

  CURL *curl = curl_easy_init();
  if (!curl) {
    return false;
  }

  std::string readBuffer;
  curl_easy_setopt(
      curl, CURLOPT_URL,
      "https://ourworldindata.org/grapher/carbon-intensity-electricity.csv");
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);
  curl_easy_setopt(curl, CURLOPT_TIMEOUT, CURL_TIMEOUT_SECONDS);
  curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 1L);
  curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, 2L);
  curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);

  CURLcode res = curl_easy_perform(curl);
  curl_easy_cleanup(curl);

  if (res != CURLE_OK) {
    return false;
  }

  try {
    std::map<std::string, std::pair<int, double>> country_latest;
    std::istringstream iss(readBuffer);
    std::string line;
    std::getline(iss, line); // Skip header

    while (std::getline(iss, line)) {
      std::istringstream line_iss(line);
      std::string entity, code, year_str, intensity_str;
      std::getline(line_iss, entity, ',');
      std::getline(line_iss, code, ',');
      std::getline(line_iss, year_str, ',');
      std::getline(line_iss, intensity_str, ',');

      if (!code.empty() && !intensity_str.empty()) {
        try {
          int year = std::stoi(year_str);
          double intensity = std::stod(intensity_str);
          auto &latest = country_latest[code];
          if (year > latest.first) {
            latest = {year, intensity};
          }
        } catch (...) {
          // Skip malformed lines
        }
      }
    }

    std::map<std::string, double> new_factors;
    for (const auto &[code, data] : country_latest) {
      std::string key = (code == "OWID_WRL") ? "WORLD" : code;
      new_factors[key] = data.second / 1000.0; // Convert g/kWh to kg/kWh
    }

    if (new_factors.find("WORLD") == new_factors.end()) {
      new_factors["WORLD"] = DEFAULT_CO2_FACTOR_KG_PER_KWH;
    }

    if (!new_factors.empty()) {
      std::unique_lock<std::shared_mutex> lock(this->co2_factors_mutex_);
      this->co2_factors_by_country_ = std::move(new_factors);
      this->co2_factors_loaded_.store(true);
      return true;
    }
  } catch (...) {
    // Failed to parse, use defaults
  }

  return false;
}

void Poirot::detect_system_info() {
  // CPU info - get real core count from /proc/cpuinfo
  this->system_info_.cpu_cores = 0;
  std::ifstream cpuinfo("/proc/cpuinfo");
  if (cpuinfo.is_open()) {
    std::string line;
    std::set<int> physical_cores;
    int current_physical_id = -1;
    int current_core_id = -1;

    while (std::getline(cpuinfo, line)) {
      if (line.find("model name") != std::string::npos) {
        size_t pos = line.find(':');
        if (pos != std::string::npos && this->system_info_.cpu_model.empty()) {
          this->system_info_.cpu_model = line.substr(pos + 2);
        }
      } else if (line.find("physical id") != std::string::npos) {
        size_t pos = line.find(':');
        if (pos != std::string::npos) {
          current_physical_id = std::stoi(line.substr(pos + 2));
        }
      } else if (line.find("core id") != std::string::npos) {
        size_t pos = line.find(':');
        if (pos != std::string::npos) {
          current_core_id = std::stoi(line.substr(pos + 2));
          // Combine physical_id and core_id to get unique physical cores
          int unique_core = (current_physical_id << 16) | current_core_id;
          physical_cores.insert(unique_core);
        }
      }
    }
    this->system_info_.cpu_cores =
        physical_cores.empty()
            ? static_cast<int>(std::thread::hardware_concurrency())
            : static_cast<int>(physical_cores.size());
  }

  if (this->system_info_.cpu_cores == 0) {
    this->system_info_.cpu_cores =
        static_cast<int>(std::thread::hardware_concurrency());
  }

  // Memory info
  struct sysinfo info;
  if (sysinfo(&info) == 0) {
    this->system_info_.mem_total_kb =
        static_cast<long>(info.totalram * info.mem_unit / 1024);
  }

  // OS info
  struct utsname uts;
  if (uname(&uts) == 0) {
    this->system_info_.os_name = uts.sysname;
    this->system_info_.os_version = uts.release;
    this->system_info_.hostname = uts.nodename;
  }

  std::ifstream os_release("/etc/os-release");
  if (os_release.is_open()) {
    std::string line;
    while (std::getline(os_release, line)) {
      if (line.find("PRETTY_NAME=") == 0) {
        this->system_info_.os_name = line.substr(13);
        this->system_info_.os_name.erase(
            std::remove(this->system_info_.os_name.begin(),
                        this->system_info_.os_name.end(), '"'),
            this->system_info_.os_name.end());
        break;
      }
    }
  }

  // TDP detection - try multiple sources in order of reliability
  this->system_info_.cpu_tdp_watts = 0.0;
  this->system_info_.rapl_available = false;

  // Also read RAPL max energy range for wraparound detection
  {
    std::ifstream max_range(
        "/sys/class/powercap/intel-rapl/intel-rapl:0/max_energy_range_uj");
    if (max_range.is_open()) {
      max_range >> this->rapl_max_energy_uj_;
    } else {
      std::ifstream amd_max_range(
          "/sys/class/powercap/amd-rapl/amd-rapl:0/max_energy_range_uj");
      if (amd_max_range.is_open()) {
        amd_max_range >> this->rapl_max_energy_uj_;
      }
    }
  }

  // 1. Intel RAPL PL1 (long-term power limit = TDP)
  for (const auto &rapl_path : {"/sys/class/powercap/intel-rapl/intel-rapl:0/"
                                "constraint_0_power_limit_uw",
                                "/sys/class/powercap/intel-rapl/intel-rapl:0/"
                                "constraint_1_power_limit_uw"}) {
    std::ifstream tdp_file(rapl_path);
    if (tdp_file.is_open()) {
      long tdp_uw = 0;
      tdp_file >> tdp_uw;
      if (tdp_uw > 0) {
        this->system_info_.cpu_tdp_watts = static_cast<double>(tdp_uw) / 1e6;
        this->system_info_.cpu_tdp_watts_type =
            poirot_msgs::msg::SystemInfo::INTEL_RAPL_TDP_TYPE;
        this->system_info_.rapl_available = true;
        break;
      }
    }
  }

  // 2. AMD RAPL power limit
  if (this->system_info_.cpu_tdp_watts == 0.0) {
    std::ifstream amd_tdp(
        "/sys/class/powercap/amd-rapl/amd-rapl:0/constraint_0_power_limit_uw");
    if (amd_tdp.is_open()) {
      long tdp_uw = 0;
      amd_tdp >> tdp_uw;
      if (tdp_uw > 0) {
        this->system_info_.cpu_tdp_watts = static_cast<double>(tdp_uw) / 1e6;
        this->system_info_.cpu_tdp_watts_type =
            poirot_msgs::msg::SystemInfo::AMD_RAPL_TDP_TYPE;
        this->system_info_.rapl_available = true;
      }
    }
  }

  // 3. Scan hwmon for power-related interfaces (zenpower, amd_energy, etc.)
  if (this->system_info_.cpu_tdp_watts == 0.0) {
    DIR *hwmon_dir = opendir("/sys/class/hwmon");
    if (hwmon_dir) {
      struct dirent *entry;
      while ((entry = readdir(hwmon_dir)) != nullptr) {
        if (entry->d_name[0] == '.') {
          continue;
        }
        std::string base = std::string("/sys/class/hwmon/") + entry->d_name;
        std::ifstream name_file(base + "/name");
        std::string name;
        if (name_file.is_open()) {
          std::getline(name_file, name);
          // Check for CPU power monitoring drivers
          if (name == "zenpower" || name == "amd_energy" ||
              name == "coretemp" || name == "k10temp" ||
              name.find("power") != std::string::npos) {
            // Try various power limit files
            for (const auto &power_file :
                 {"power1_cap", "power1_max", "power1_cap_max",
                  "power1_rated_max"}) {
              std::ifstream pf(base + "/" + power_file);
              if (pf.is_open()) {
                long power_uw = 0;
                pf >> power_uw;
                if (power_uw > 0) {
                  this->system_info_.cpu_tdp_watts =
                      static_cast<double>(power_uw) / 1e6;
                  this->system_info_.cpu_tdp_watts_type =
                      poirot_msgs::msg::SystemInfo::HWMON_RAPL_TDP_TYPE;
                  break;
                }
              }
            }
            if (this->system_info_.cpu_tdp_watts > 0) {
              break;
            }
          }
        }
      }
      closedir(hwmon_dir);
    }
  }

  // 4. Try thermal zone power budget
  if (this->system_info_.cpu_tdp_watts == 0.0) {
    DIR *thermal_dir = opendir("/sys/class/thermal");
    if (thermal_dir) {
      struct dirent *entry;
      while ((entry = readdir(thermal_dir)) != nullptr) {
        if (strncmp(entry->d_name, "cooling_device", 14) == 0) {
          std::string base = std::string("/sys/class/thermal/") + entry->d_name;
          std::ifstream type_file(base + "/type");
          std::string type;
          if (type_file.is_open()) {
            std::getline(type_file, type);
            if (type.find("Processor") != std::string::npos ||
                type.find("processor") != std::string::npos) {
              std::ifstream max_state(base + "/max_state");
              if (max_state.is_open()) {
                long max_pstate = 0;
                max_state >> max_pstate;
                // P-states roughly correlate to power levels
                if (max_pstate > 0) {
                  // Read sustainable power from thermal zone if available
                  std::ifstream sustainable_power(
                      "/sys/class/thermal/thermal_zone0/sustainable_power");
                  if (sustainable_power.is_open()) {
                    long power_mw = 0;
                    sustainable_power >> power_mw;
                    if (power_mw > 0) {
                      this->system_info_.cpu_tdp_watts =
                          static_cast<double>(power_mw) / 1000.0;
                    }
                  }
                  if (this->system_info_.cpu_tdp_watts == 0.0) {
                    // Estimate based on typical TDP per P-state
                    this->system_info_.cpu_tdp_watts =
                        static_cast<double>(max_pstate) * 10.0;
                  }
                  this->system_info_.cpu_tdp_watts_type =
                      poirot_msgs::msg::SystemInfo::THERMAL_POWER_TDP_TYPE;
                  break;
                }
              }
            }
          }
        }
      }
      closedir(thermal_dir);
    }
  }

  // 5. Estimate from CPU frequency and core count (physics-based)
  if (this->system_info_.cpu_tdp_watts == 0.0) {
    double max_freq_ghz = 0.0;
    std::ifstream freq_file(
        "/sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_max_freq");
    if (freq_file.is_open()) {
      long freq_khz = 0;
      freq_file >> freq_khz;
      max_freq_ghz = static_cast<double>(freq_khz) / 1e6;
    }

    if (max_freq_ghz > 0 && this->system_info_.cpu_cores > 0) {
      // Empirical formula: TDP ≈ cores × freq_ghz × power_per_core_per_ghz
      // Typical values: ~8-12W per core per GHz for modern CPUs
      constexpr double power_factor =
          10.0; // W per core per GHz (conservative estimate)
      this->system_info_.cpu_tdp_watts =
          this->system_info_.cpu_cores * max_freq_ghz * power_factor * 0.5;
      // Cap at reasonable values
      this->system_info_.cpu_tdp_watts =
          std::min(std::max(this->system_info_.cpu_tdp_watts, MIN_TDP_WATTS),
                   MAX_TDP_WATTS);
      this->system_info_.cpu_tdp_watts_type =
          poirot_msgs::msg::SystemInfo::CPU_CORES_FREQUENCY_TYPE;
    }
  }

  // 6. Final fallback: estimate from core count alone
  if (this->system_info_.cpu_tdp_watts == 0.0) {
    // Typical desktop: ~10-15W per core, laptop: ~5-8W per core
    constexpr double watts_per_core = 12.0;
    this->system_info_.cpu_tdp_watts =
        this->system_info_.cpu_cores * watts_per_core;
    this->system_info_.cpu_tdp_watts =
        std::min(std::max(this->system_info_.cpu_tdp_watts, MIN_TDP_WATTS),
                 MAX_TDP_WATTS);
    this->system_info_.cpu_tdp_watts_type =
        poirot_msgs::msg::SystemInfo::CPU_CORES_TYPE;
  }

  // CO2 factor from timezone
  this->system_info_.country_code = this->get_country_from_timezone();
  this->system_info_.co2_factor_kg_per_kwh =
      this->get_co2_factor_for_country(this->system_info_.country_code);
}

void Poirot::search_hwmon_paths() {
  if (this->hwmon_paths_searched_) {
    return;
  }
  this->hwmon_paths_searched_ = true;

  DIR *hwmon_dir = opendir("/sys/class/hwmon");
  if (!hwmon_dir) {
    return;
  }

  struct dirent *entry;
  while ((entry = readdir(hwmon_dir)) != nullptr) {
    if (entry->d_name[0] == '.') {
      continue;
    }

    std::string base = std::string("/sys/class/hwmon/") + entry->d_name;
    std::ifstream name_file(base + "/name");
    std::string name;
    if (!name_file.is_open()) {
      continue;
    }

    std::getline(name_file, name);

    // Check for energy monitoring drivers
    if (name == "zenpower" || name == "amd_energy" ||
        name.find("energy") != std::string::npos) {
      for (int i = 1; i <= 10; ++i) {
        std::string energy_path =
            base + "/energy" + std::to_string(i) + "_input";
        std::ifstream ef(energy_path);
        if (ef.is_open()) {
          this->cached_hwmon_energy_path_ = energy_path;
          break;
        }
      }
    }

    // Check for power monitoring drivers
    if (name == "zenpower" || name == "amd_energy" || name == "coretemp" ||
        name == "k10temp" || name.find("power") != std::string::npos) {
      for (int i = 1; i <= 10; ++i) {
        std::string power_path = base + "/power" + std::to_string(i) + "_input";
        std::ifstream pf(power_path);
        if (pf.is_open()) {
          this->cached_hwmon_power_path_ = power_path;
          break;
        }
      }
    }

    if (!this->cached_hwmon_energy_path_.empty() &&
        !this->cached_hwmon_power_path_.empty()) {
      break;
    }
  }
  closedir(hwmon_dir);
}

std::string Poirot::get_country_from_timezone() {
  std::string tz;

  // 1. Try TZ environment variable
  const char *tz_env = std::getenv("TZ");
  if (tz_env != nullptr && tz_env[0] != '\0') {
    tz = tz_env;
  }

  // 2. Try /etc/timezone (Debian/Ubuntu)
  if (tz.empty()) {
    std::ifstream tz_file("/etc/timezone");
    if (tz_file.is_open()) {
      std::getline(tz_file, tz);
    }
  }

  // 3. Try reading /etc/localtime symlink target
  if (tz.empty()) {
    char buf[256];
    ssize_t len = readlink("/etc/localtime", buf, sizeof(buf) - 1);
    if (len > 0) {
      buf[len] = '\0';
      std::string target(buf);
      // Format: /usr/share/zoneinfo/Region/City
      size_t pos = target.find("zoneinfo/");
      if (pos != std::string::npos) {
        tz = target.substr(pos + 9);
      }
    }
  }

  if (tz.empty()) {
    return "WORLD";
  }

  // Parse zone.tab to get country code from timezone
  // zone.tab format: CC coordinates TZ/Name comments
  std::ifstream zone_tab("/usr/share/zoneinfo/zone.tab");
  if (zone_tab.is_open()) {
    std::string line;
    while (std::getline(zone_tab, line)) {
      if (line.empty() || line[0] == '#') {
        continue;
      }

      std::istringstream iss(line);
      std::string country_code, coordinates, timezone;
      iss >> country_code >> coordinates >> timezone;

      if (timezone == tz) {
        return country_code;
      }
    }
  }

  // Fallback: try zone1970.tab (newer format)
  std::ifstream zone1970_tab("/usr/share/zoneinfo/zone1970.tab");
  if (zone1970_tab.is_open()) {
    std::string line;
    while (std::getline(zone1970_tab, line)) {
      if (line.empty() || line[0] == '#') {
        continue;
      }

      std::istringstream iss(line);
      std::string countries, coordinates, timezone;
      iss >> countries >> coordinates >> timezone;

      if (timezone == tz) {
        // countries can be comma-separated, return first one
        size_t comma = countries.find(',');
        return comma != std::string::npos ? countries.substr(0, comma)
                                          : countries;
      }
    }
  }

  return "WORLD";
}

double Poirot::get_co2_factor_for_country(const std::string &country) {
  std::shared_lock<std::shared_mutex> lock(this->co2_factors_mutex_);

  if (!this->co2_factors_loaded_.load()) {
    return DEFAULT_CO2_FACTOR_KG_PER_KWH;
  }

  auto it = this->co2_factors_by_country_.find(country);
  if (it != this->co2_factors_by_country_.end()) {
    return it->second;
  }

  auto world_it = this->co2_factors_by_country_.find("WORLD");
  return world_it != this->co2_factors_by_country_.end()
             ? world_it->second
             : DEFAULT_CO2_FACTOR_KG_PER_KWH;
}

// ============================================================================
// Thread-Local Context Management
// ============================================================================
ThreadProfilingContext &Poirot::get_thread_context() {
  std::lock_guard<std::mutex> lock(this->contexts_mutex_);
  auto thread_id = std::this_thread::get_id();
  auto &ctx = this->thread_contexts_[thread_id];
  ctx.thread_id = thread_id;
  return ctx;
}

// ============================================================================
// Thread-Aware Measurement Functions
// ============================================================================
double Poirot::read_thread_cpu_time_us() {
  struct timespec ts;
  if (clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts) == 0) {
    return static_cast<double>(ts.tv_sec) * 1e6 +
           static_cast<double>(ts.tv_nsec) / 1e3;
  }
  return 0.0;
}

double Poirot::read_process_cpu_time_us() {
  // Use CLOCK_PROCESS_CPUTIME_ID for total process CPU time
  struct timespec ts;
  if (clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &ts) == 0) {
    return static_cast<double>(ts.tv_sec) * 1e6 +
           static_cast<double>(ts.tv_nsec) / 1e3;
  }
  return 0.0;
}

double Poirot::read_system_cpu_time_us() {
  // Read system-wide CPU time from /proc/stat
  // This gives us total CPU time across all cores and all processes
  // Format: cpu user nice system idle iowait irq softirq steal guest guest_nice
  std::ifstream stat("/proc/stat");
  if (!stat.is_open()) {
    return 0.0;
  }

  std::string line;
  std::getline(stat, line);

  // Parse the "cpu" line (aggregate of all CPUs)
  if (line.compare(0, 4, "cpu ") != 0) {
    return 0.0;
  }

  std::istringstream iss(line.substr(4)); // Skip "cpu "
  unsigned long long user = 0, nice = 0, system = 0, idle = 0;
  unsigned long long iowait = 0, irq = 0, softirq = 0, steal = 0;

  iss >> user >> nice >> system >> idle >> iowait >> irq >> softirq >> steal;

  // Total active CPU time (excluding idle and iowait)
  // This represents actual work being done by all processes
  unsigned long long total_active =
      user + nice + system + irq + softirq + steal;

  // Convert from jiffies to microseconds
  long sc_clk_tck = sysconf(_SC_CLK_TCK);
  if (sc_clk_tck <= 0) {
    sc_clk_tck = 100; // Fallback to common default
  }

  return static_cast<double>(total_active) * 1e6 /
         static_cast<double>(sc_clk_tck);
}

double Poirot::calculate_thread_energy_uj(double thread_cpu_delta_us,
                                          double process_cpu_delta_us,
                                          double system_cpu_delta_us,
                                          double total_energy_delta_uj) {
  // Thread-level energy estimation using hierarchical CPU time attribution
  //
  // This method uses a two-level attribution model:
  // 1. First, determine what fraction of package energy our process consumed
  //    based on process CPU time vs system-wide CPU time
  // 2. Then, determine what fraction of that process energy this thread
  // consumed
  //    based on thread CPU time vs process CPU time
  //
  // Formula:
  //   process_energy = (process_cpu_delta / system_cpu_delta) * package_energy
  //   thread_energy  = (thread_cpu_delta / process_cpu_delta) * process_energy
  //
  // Simplified:
  //   thread_energy = (thread_cpu_delta / system_cpu_delta) * package_energy
  //
  // Why system CPU time matters:
  // - RAPL measures energy for the entire CPU package (all cores, all
  // processes)
  // - If other processes are running, they consume part of that energy
  // - Using only process CPU time would over-attribute energy to our threads
  //
  // Fallback strategy:
  // - If system CPU is unavailable, fall back to process-level attribution
  // - If process CPU is unavailable, attribute all energy to the thread

  if (total_energy_delta_uj <= 0.0) {
    return 0.0;
  }

  if (thread_cpu_delta_us <= 0.0) {
    // Thread consumed no CPU time, minimal energy for overhead
    constexpr double MIN_ATTRIBUTION_FACTOR = 0.001; // 0.1% minimum
    return total_energy_delta_uj * MIN_ATTRIBUTION_FACTOR;
  }

  double cpu_ratio = 0.0;

  // Prefer system-wide attribution (most accurate)
  if (system_cpu_delta_us > 0.0) {
    // Direct attribution: thread's share of system-wide CPU = thread's share of
    // package energy
    cpu_ratio = thread_cpu_delta_us / system_cpu_delta_us;
  } else if (process_cpu_delta_us > 0.0) {
    // Fallback: use process-level attribution (less accurate when other
    // processes are active)
    cpu_ratio = thread_cpu_delta_us / process_cpu_delta_us;
  } else {
    // No reference available, attribute all measured energy to thread
    return total_energy_delta_uj;
  }

  // Clamp ratio to [0, 1] to handle measurement anomalies
  // In rare cases, timing differences can cause ratio > 1
  cpu_ratio = std::max(0.0, std::min(1.0, cpu_ratio));

  return total_energy_delta_uj * cpu_ratio;
}

long Poirot::read_thread_memory_kb() {
  // Read thread-specific memory from /proc/self/task/[tid]/status
  pid_t tid = static_cast<pid_t>(syscall(SYS_gettid));
  std::string path = "/proc/self/task/" + std::to_string(tid) + "/status";
  std::ifstream file(path);

  if (!file.is_open()) {
    // Fallback to process-level memory
    file.open("/proc/self/status");
    if (!file.is_open()) {
      return 0;
    }
  }

  std::string line;
  while (std::getline(file, line)) {
    if (line.compare(0, 6, "VmRSS:") == 0) {
      std::istringstream iss(line);
      std::string label;
      long value = 0;
      iss >> label >> value;
      return value;
    }
  }
  return 0;
}

void Poirot::read_thread_io_bytes(long &read_bytes, long &write_bytes) {
  read_bytes = 0;
  write_bytes = 0;

  pid_t tid = static_cast<pid_t>(syscall(SYS_gettid));
  std::string path = "/proc/self/task/" + std::to_string(tid) + "/io";
  std::ifstream file(path);
  if (!file.is_open()) {
    // Fallback to process-level I/O
    file.open("/proc/self/io");
    if (!file.is_open()) {
      return;
    }
  }

  std::string line;
  while (std::getline(file, line)) {
    if (line.compare(0, 11, "read_bytes:") == 0) {
      try {
        read_bytes = std::stol(line.substr(12));
      } catch (...) {
        read_bytes = 0;
      }
    } else if (line.compare(0, 12, "write_bytes:") == 0) {
      try {
        write_bytes = std::stol(line.substr(13));
      } catch (...) {
        write_bytes = 0;
      }
    }
  }
}

long Poirot::read_thread_ctx_switches() {
  pid_t tid = static_cast<pid_t>(syscall(SYS_gettid));
  std::string path = "/proc/self/task/" + std::to_string(tid) + "/status";
  std::ifstream file(path);
  if (!file.is_open()) {
    file.open("/proc/self/status");
    if (!file.is_open()) {
      return 0;
    }
  }

  long total = 0;
  std::string line;
  while (std::getline(file, line)) {
    if (line.compare(0, 24, "voluntary_ctxt_switches:") == 0) {
      std::istringstream iss(line.substr(24));
      long val = 0;
      iss >> val;
      total += val;
    } else if (line.compare(0, 27, "nonvoluntary_ctxt_switches:") == 0) {
      std::istringstream iss(line.substr(27));
      long val = 0;
      iss >> val;
      total += val;
    }
  }
  return total;
}

double Poirot::get_battery_power_w() {
  DIR *dir = opendir("/sys/class/power_supply");
  if (!dir) {
    return -1.0;
  }

  struct dirent *entry;
  while ((entry = readdir(dir)) != nullptr) {
    if (strncmp(entry->d_name, "BAT", 3) == 0) {
      std::string base =
          std::string("/sys/class/power_supply/") + entry->d_name;

      // Try power_now first (direct power measurement)
      std::ifstream power_file(base + "/power_now");
      if (power_file.is_open()) {
        long power_uw = 0;
        power_file >> power_uw;
        if (power_uw > 0) {
          closedir(dir);
          return static_cast<double>(power_uw) / 1e6;
        }
      }

      // Fallback: calculate from voltage and current
      std::ifstream voltage_file(base + "/voltage_now");
      std::ifstream current_file(base + "/current_now");
      if (voltage_file.is_open() && current_file.is_open()) {
        long voltage_uv = 0;
        long current_ua = 0;
        voltage_file >> voltage_uv;
        current_file >> current_ua;
        if (voltage_uv > 0 && current_ua > 0) {
          // P = V * I (both in micro units -> result in pW, divide by 1e12 for
          // W)
          double power_w = (static_cast<double>(voltage_uv) *
                            static_cast<double>(current_ua)) /
                           1e12;
          closedir(dir);
          return power_w;
        }
      }
    }
  }
  closedir(dir);
  return -1.0;
}

double Poirot::read_energy_uj() {
  std::lock_guard<std::mutex> lock(this->energy_mutex_);

  // Helper lambda to read RAPL energy with wraparound detection
  auto read_rapl_energy = [this](const std::string &path) -> double {
    std::ifstream file(path);
    if (!file.is_open()) {
      return -1.0;
    }

    double current_energy = 0.0;
    file >> current_energy;
    if (current_energy <= 0) {
      return -1.0;
    }

    // Handle RAPL counter wraparound
    if (this->last_rapl_energy_uj_ > 0) {
      if (current_energy < this->last_rapl_energy_uj_) {
        // Counter wrapped around
        if (this->rapl_max_energy_uj_ > 0) {
          double delta =
              (this->rapl_max_energy_uj_ - this->last_rapl_energy_uj_) +
              current_energy;
          this->accumulated_energy_uj_ += delta;
        }
      } else {
        this->accumulated_energy_uj_ +=
            (current_energy - this->last_rapl_energy_uj_);
      }
    }
    this->last_rapl_energy_uj_ = current_energy;
    return this->accumulated_energy_uj_;
  };

  // 1. Try Intel RAPL (most accurate for Intel CPUs)
  double energy =
      read_rapl_energy("/sys/class/powercap/intel-rapl/intel-rapl:0/energy_uj");
  if (energy >= 0) {
    return energy;
  }

  // 2. Try AMD RAPL
  energy =
      read_rapl_energy("/sys/class/powercap/amd-rapl/amd-rapl:0/energy_uj");
  if (energy >= 0) {
    return energy;
  }

  // 3. Try cached hwmon energy path
  if (!this->cached_hwmon_energy_path_.empty()) {
    std::ifstream file(this->cached_hwmon_energy_path_);
    if (file.is_open()) {
      double hwmon_energy = 0.0;
      file >> hwmon_energy;
      if (hwmon_energy > 0) {
        // Handle wraparound for hwmon too
        if (this->last_rapl_energy_uj_ > 0 &&
            hwmon_energy < this->last_rapl_energy_uj_) {
          // Assume 32-bit counter wraparound
          constexpr double max_32bit = 4294967295.0;
          double delta =
              (max_32bit - this->last_rapl_energy_uj_) + hwmon_energy;
          this->accumulated_energy_uj_ += delta;
        } else if (this->last_rapl_energy_uj_ > 0) {
          this->accumulated_energy_uj_ +=
              (hwmon_energy - this->last_rapl_energy_uj_);
        }
        this->last_rapl_energy_uj_ = hwmon_energy;
        return this->accumulated_energy_uj_;
      }
    }
  }

  // 4. Estimate energy based on power measurements and elapsed time
  auto now = std::chrono::high_resolution_clock::now();

  if (this->last_energy_read_time_.time_since_epoch().count() == 0) {
    this->last_energy_read_time_ = now;
    return this->accumulated_energy_uj_;
  }

  double elapsed_us =
      static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
                              now - this->last_energy_read_time_)
                              .count());
  this->last_energy_read_time_ = now;

  if (elapsed_us <= 0) {
    return this->accumulated_energy_uj_;
  }

  // Try to get actual power measurement
  double power_w = this->get_battery_power_w();

  // Try hwmon power sensors
  if (power_w <= 0 && !this->cached_hwmon_power_path_.empty()) {
    std::ifstream file(this->cached_hwmon_power_path_);
    if (file.is_open()) {
      long power_uw = 0;
      file >> power_uw;
      if (power_uw > 0) {
        power_w = static_cast<double>(power_uw) / 1e6;
      }
    }
  }

  // Estimate from TDP and CPU utilization
  if (power_w <= 0) {
    double cpu_pct = this->process_info_.process_cpu_percent;

    // Apply utilization factor with idle power baseline
    double util_factor =
        IDLE_POWER_FACTOR + (1.0 - IDLE_POWER_FACTOR) * (cpu_pct / 100.0);
    util_factor = std::max(IDLE_POWER_FACTOR, std::min(1.0, util_factor));

    power_w = this->system_info_.cpu_tdp_watts * util_factor;
  }

  // Calculate energy: E = P * t (power in W, time in us -> energy in uJ)
  this->accumulated_energy_uj_ += power_w * elapsed_us;
  return this->accumulated_energy_uj_;
}

// ============================================================================
// System-Level Measurements
// ============================================================================
double Poirot::read_process_cpu_percent() {
  std::lock_guard<std::mutex> lock(this->cpu_read_mutex_);

  std::ifstream stat("/proc/self/stat");
  if (!stat.is_open()) {
    return 0.0;
  }

  std::string line;
  std::getline(stat, line);

  // Parse /proc/self/stat - handle process names with spaces/parentheses
  // Format: pid (comm) state ppid pgrp session tty_nr tpgid flags minflt ...
  // Fields 14 (utime) and 15 (stime) are 0-indexed from the end of comm
  size_t comm_start = line.find('(');
  size_t comm_end = line.rfind(')');
  if (comm_start == std::string::npos || comm_end == std::string::npos) {
    return 0.0;
  }

  std::string after_comm = line.substr(comm_end + 2); // Skip ") "
  std::istringstream iss(after_comm);
  std::string token;

  // Skip state(1), ppid(2), pgrp(3), session(4), tty_nr(5), tpgid(6),
  // flags(7), minflt(8), cminflt(9), majflt(10), cmajflt(11)
  // Then read utime(12) and stime(13) - these are indices after comm
  unsigned long long utime = 0;
  unsigned long long stime = 0;

  for (int i = 1; i <= 13; ++i) {
    if (!(iss >> token)) {
      return 0.0;
    }
    if (i == 12) {
      try {
        utime = std::stoull(token);
      } catch (...) {
        return 0.0;
      }
    } else if (i == 13) {
      try {
        stime = std::stoull(token);
      } catch (...) {
        return 0.0;
      }
    }
  }

  auto now = std::chrono::high_resolution_clock::now();
  double time_diff =
      std::chrono::duration<double>(now - this->prev_cpu_read_time_).count();

  double pct = 0.0;
  unsigned long long prev_cpu = this->prev_process_cpu_.load();
  if (time_diff > 0 && prev_cpu > 0) {
    unsigned long long cpu_diff = (utime + stime) - prev_cpu;
    long sc_clk_tck = sysconf(_SC_CLK_TCK);
    double cpu_seconds =
        static_cast<double>(cpu_diff) / static_cast<double>(sc_clk_tck);
    pct = (cpu_seconds / time_diff) * 100.0;
  }

  this->prev_process_cpu_.store(utime + stime);
  this->prev_cpu_read_time_ = now;
  return pct;
}

int Poirot::read_process_thread_count() {
  std::ifstream status("/proc/self/status");
  if (!status.is_open()) {
    return 1;
  }

  std::string line;
  while (std::getline(status, line)) {
    if (line.compare(0, 8, "Threads:") == 0) {
      std::istringstream iss(line.substr(8));
      int threads = 1;
      iss >> threads;
      return threads;
    }
  }
  return 1;
}

void Poirot::read_process_data() {
  this->process_info_.pid = getpid();

  this->process_info_.process_cpu_percent = this->read_process_cpu_percent();
  this->process_info_.process_mem_kb = this->read_thread_memory_kb();

  long io_read = 0;
  long io_write = 0;
  this->read_thread_io_bytes(io_read, io_write);
  this->process_info_.process_io_bytes = io_read + io_write;
  this->process_info_.process_threads = this->read_process_thread_count();
}

// ============================================================================
// Profiling Control (Thread-Safe)
// ============================================================================
void Poirot::start_profiling(const std::string &function_name) {
  this->read_process_data();

  // Get thread-local context
  ThreadProfilingContext &ctx = this->get_thread_context();
  ctx.function_name = function_name;
  ctx.start_time = std::chrono::high_resolution_clock::now();
  ctx.start_cpu_time_us = this->read_thread_cpu_time_us();
  ctx.start_process_cpu_time_us = this->read_process_cpu_time_us();
  ctx.start_system_cpu_time_us =
      this->read_system_cpu_time_us(); // System-wide for accurate attribution
  ctx.start_memory_kb = this->read_thread_memory_kb();
  this->read_thread_io_bytes(ctx.start_io_read_bytes, ctx.start_io_write_bytes);
  ctx.start_ctx_switches = this->read_thread_ctx_switches();
  ctx.start_energy_uj = this->read_energy_uj();
}

void Poirot::stop_profiling() {
  this->read_process_data();

  auto end_time = std::chrono::high_resolution_clock::now();
  double end_cpu_time_us = this->read_thread_cpu_time_us();
  double end_process_cpu_time_us = this->read_process_cpu_time_us();
  double end_system_cpu_time_us =
      this->read_system_cpu_time_us(); // System-wide for accurate attribution
  long end_memory_kb = this->read_thread_memory_kb();
  long end_io_read_bytes = 0;
  long end_io_write_bytes = 0;
  this->read_thread_io_bytes(end_io_read_bytes, end_io_write_bytes);
  long end_ctx_switches = this->read_thread_ctx_switches();
  double end_energy_uj = this->read_energy_uj();

  // Get thread-local context
  ThreadProfilingContext &ctx = this->get_thread_context();

  // Calculate deltas
  double thread_cpu_delta_us = end_cpu_time_us - ctx.start_cpu_time_us;
  double process_cpu_delta_us =
      end_process_cpu_time_us - ctx.start_process_cpu_time_us;
  double system_cpu_delta_us =
      end_system_cpu_time_us - ctx.start_system_cpu_time_us;
  double total_energy_delta_uj = end_energy_uj - ctx.start_energy_uj;

  // Calculate thread-level energy using hierarchical CPU time attribution
  double thread_energy_uj = this->calculate_thread_energy_uj(
      thread_cpu_delta_us, process_cpu_delta_us, system_cpu_delta_us,
      total_energy_delta_uj);

  poirot_msgs::msg::FunctionCall call;
  call.timestamp = rclcpp::Clock().now();
  call.data.wall_time_us =
      static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
                              end_time - ctx.start_time)
                              .count());
  call.data.cpu_time_us = thread_cpu_delta_us;
  call.data.memory_kb = end_memory_kb - ctx.start_memory_kb;
  call.data.io_read_bytes = end_io_read_bytes - ctx.start_io_read_bytes;
  call.data.io_write_bytes = end_io_write_bytes - ctx.start_io_write_bytes;
  call.data.ctx_switches = end_ctx_switches - ctx.start_ctx_switches;
  call.data.energy_uj =
      thread_energy_uj; // Thread-level energy using system-wide attribution

  // Calculate CO2 in micrograms: energy_uj * co2_factor_kg_per_kwh * conversion
  // energy_uj / 1e6 = J, / 3600 = Wh, / 1000 = kWh
  // co2_factor is kg/kWh, * 1e9 = ug
  constexpr double UJ_TO_KWH = 1.0 / 1e6 / 3600.0 / 1000.0;
  constexpr double KG_TO_UG = 1e9;
  double energy_kwh = call.data.energy_uj * UJ_TO_KWH;
  call.data.co2_ug =
      energy_kwh * this->system_info_.co2_factor_kg_per_kwh * KG_TO_UG;

  {
    std::unique_lock<std::shared_mutex> lock(this->statistics_mutex_);

    auto &stats = this->statistics_[ctx.function_name];
    stats.name = ctx.function_name;
    stats.call_count++;
    stats.total_data.wall_time_us += call.data.wall_time_us;
    stats.total_data.cpu_time_us += call.data.cpu_time_us;
    stats.total_data.io_read_bytes += call.data.io_read_bytes;
    stats.total_data.io_write_bytes += call.data.io_write_bytes;
    stats.total_data.energy_uj += call.data.energy_uj;
    stats.total_data.co2_ug += call.data.co2_ug;
    stats.last_call = call;
  }

  if (this->verbose_.load()) {
    std::cout << "[PROFILE] " << ctx.function_name
              << " | Wall: " << call.data.wall_time_us << "us"
              << " | CPU: " << call.data.cpu_time_us << "us"
              << " | Mem: " << call.data.memory_kb << "KB"
              << " | IO R/W: " << call.data.io_read_bytes << "/"
              << call.data.io_write_bytes << "B"
              << " | CtxSw: " << call.data.ctx_switches
              << " | Energy: " << call.data.energy_uj << "uJ"
              << " | CO2: " << call.data.co2_ug << "ug" << std::endl;
  }

  this->publish_stats(ctx.function_name);
}

// ============================================================================
// Getters (Return copies for thread safety)
// ============================================================================
std::map<std::string, poirot_msgs::msg::FunctionStats>
Poirot::get_statistics() const {
  std::shared_lock<std::shared_mutex> lock(this->statistics_mutex_);
  return this->statistics_;
}

poirot_msgs::msg::SystemInfo Poirot::get_system_info() const {
  return this->system_info_;
}

poirot_msgs::msg::ProcessInfo Poirot::get_process_info() const {
  return this->process_info_;
}

// ============================================================================
// Configuration
// ============================================================================
void Poirot::set_co2_factor(double factor) {
  this->system_info_.co2_factor_kg_per_kwh = factor;
}

void Poirot::set_verbose(bool verbose) {
  get_instance().verbose_.store(verbose);
}

// ============================================================================
// Output
// ============================================================================
void Poirot::print_summary() const {
  if (!this->verbose_.load()) {
    return;
  }

  std::shared_lock<std::shared_mutex> lock(this->statistics_mutex_);
  if (this->statistics_.empty()) {
    return;
  }

  std::cout << "\n=========================================="
            << "=========================\n";
  std::cout << "           ROS 2 FUNCTION PROFILING SUMMARY\n";
  std::cout << "============================================"
            << "=======================\n";
  std::cout << std::fixed << std::setprecision(2);

  double total_energy_uj = 0;
  for (const auto &[name, s] : this->statistics_) {
    std::cout << "\n[" << name << "]\n";
    std::cout << "  Calls:     " << s.call_count << "\n";
    std::cout << "  Wall Time: " << s.total_data.wall_time_us / 1000.0
              << " ms total, " << s.total_data.wall_time_us << " us\n";
    std::cout << "  CPU Time:  " << s.total_data.cpu_time_us / 1000.0
              << " ms total\n";
    std::cout << "  Energy:    " << s.total_data.energy_uj / 1000.0 << " mJ\n";
    total_energy_uj += s.total_data.energy_uj;
  }

  constexpr double UJ_TO_KWH = 1.0 / 1e6 / 3600000.0;
  double energy_kwh = total_energy_uj * UJ_TO_KWH;
  double co2_g = energy_kwh * this->system_info_.co2_factor_kg_per_kwh * 1000.0;
  std::cout << "\nTotal Energy: " << total_energy_uj / 1e6 << " J"
            << " | CO2: " << co2_g * 1000.0 << " mg\n";
  std::cout << "============================================"
            << "=======================\n";
}

void Poirot::print_system_info() {
  const Poirot &instance = get_instance();
  std::cout << "\n=========================================="
            << "=========================\n";
  std::cout << "              SYSTEM INFORMATION\n";
  std::cout << "============================================"
            << "=======================\n";
  std::cout << "CPU:      " << instance.system_info_.cpu_model << "\n";
  std::cout << "Cores:    " << instance.system_info_.cpu_cores << "\n";
  std::cout << "Memory:   " << instance.system_info_.mem_total_kb / 1024
            << " MB\n";
  std::cout << "OS:       " << instance.system_info_.os_name << "\n";
  std::cout << "Hostname: " << instance.system_info_.hostname << "\n";
  std::cout << "RAPL:     "
            << (instance.system_info_.rapl_available ? "Yes" : "No") << "\n";
  std::cout << "TDP:      " << instance.system_info_.cpu_tdp_watts << " W\n";
  std::cout << "TDP Type: "
            << static_cast<int>(instance.system_info_.cpu_tdp_watts_type)
            << "\n";
  std::cout << "Country:  " << instance.system_info_.country_code << "\n";
  std::cout << "CO2:      " << instance.system_info_.co2_factor_kg_per_kwh
            << " kg/kWh\n";
  std::cout << "============================================"
            << "=======================\n";
}

// ============================================================================
// ROS 2 Publishing
// ============================================================================
void Poirot::publish_stats(const std::string &function_name) {
  std::shared_lock<std::shared_mutex> lock(this->statistics_mutex_);

  auto msg = poirot_msgs::msg::ProfilingData();
  msg.system_info = this->system_info_;
  msg.process_info = this->process_info_;
  msg.function = this->statistics_[function_name];
  msg.timestamp = msg.function.last_call.timestamp;

  this->profiling_data_publisher_->publish(msg);
}

// ============================================================================
// ScopedPoirot
// ============================================================================
std::string ScopedPoirot::extract_function_name(const char *pretty_function) {
  std::string func(pretty_function);
  size_t paren_pos = func.find('(');
  if (paren_pos != std::string::npos) {
    func = func.substr(0, paren_pos);
  }

  size_t space_pos = func.rfind(' ');
  if (space_pos != std::string::npos) {
    func = func.substr(space_pos + 1);
  }

  size_t lambda_pos = func.find("<lambda");
  if (lambda_pos != std::string::npos) {
    std::string scope = func.substr(0, lambda_pos);
    while (!scope.empty() && (scope.back() == ':' || scope.back() == ' ')) {
      scope.pop_back();
    }
    func = scope.empty() ? "lambda" : scope + "::lambda";
  }

  return func.empty() ? "unknown" : func;
}

ScopedPoirot::ScopedPoirot(Poirot &profiler, const char *func, const char *file,
                           int line)
    : profiler_(profiler) {
  (void)file;
  (void)line;
  profiler_.start_profiling(extract_function_name(func));
}

ScopedPoirot::~ScopedPoirot() { profiler_.stop_profiling(); }

// ============================================================================
// Static Instance
// ============================================================================
Poirot &Poirot::get_instance() {
  static Poirot profiler;
  return profiler;
}
