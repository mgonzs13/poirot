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
#include <vector>

#include "poirot_msgs/msg/function_call.hpp"
#include "poirot_msgs/msg/function_stats.hpp"
#include "poirot_msgs/msg/profiling_data.hpp"

#include "poirot/poirot.hpp"

using namespace poirot;

// ============================================================================
// Constants
// ============================================================================
/// @brief Default CO2 factor in kg per kWh
static constexpr double DEFAULT_CO2_FACTOR_KG_PER_KWH = 0.475;
/// @brief Fallback minimum TDP in watts if system detection fails
static constexpr double FALLBACK_MIN_TDP_WATTS = 15.0;
/// @brief Fallback maximum TDP in watts if system detection fails
static constexpr double FALLBACK_MAX_TDP_WATTS = 400.0;
/// @brief Fallback idle power factor if measurement fails
static constexpr double FALLBACK_IDLE_POWER_FACTOR = 0.15;
/// @brief Fallback watts per core per GHz if calculation fails
static constexpr double FALLBACK_WATTS_PER_GHZ = 4.0;
/// @brief Fallback minimum watts per GHz for validation
static constexpr double FALLBACK_MIN_WATTS_PER_GHZ = 2.0;
/// @brief Fallback maximum watts per GHz for validation
static constexpr double FALLBACK_MAX_WATTS_PER_GHZ = 20.0;
/// @brief Fallback power per core per GHz if all measurements fail
static constexpr double FALLBACK_POWER_PER_CORE_PER_GHZ = 10.0;
/// @brief Fallback watts per core if all measurements fail
static constexpr double FALLBACK_WATTS_PER_CORE = 12.0;
/// @brief CURL timeout in seconds for downloading CO2 factors
static constexpr long CURL_TIMEOUT_SECONDS = 10L;

// ============================================================================
// Helper Functions
// ============================================================================
long Poirot::read_sysfs_long(const std::string &path, long default_value) {
  std::ifstream file(path);
  if (file.is_open()) {
    long value = default_value;
    file >> value;
    return value;
  }
  return default_value;
}

double Poirot::read_sysfs_double(const std::string &path,
                                 double default_value) {
  std::ifstream file(path);
  if (file.is_open()) {
    double value = default_value;
    file >> value;
    return value;
  }
  return default_value;
}

std::string Poirot::read_sysfs_string(const std::string &path) {
  std::ifstream file(path);
  if (file.is_open()) {
    std::string value;
    std::getline(file, value);
    return value;
  }
  return "";
}

std::string Poirot::get_thread_status_path(const std::string &filename) {
  pid_t tid = static_cast<pid_t>(syscall(SYS_gettid));
  std::string thread_path =
      "/proc/self/task/" + std::to_string(tid) + "/" + filename;
  std::ifstream test(thread_path);
  if (test.is_open()) {
    return thread_path;
  }
  return "/proc/self/" + filename;
}

double Poirot::read_rapl_power_limit_w() {
  static const std::vector<std::string> rapl_paths = {
      "/sys/class/powercap/intel-rapl/intel-rapl:0/constraint_0_power_limit_uw",
      "/sys/class/powercap/intel-rapl/intel-rapl:0/constraint_1_power_limit_uw",
      "/sys/class/powercap/amd-rapl/amd-rapl:0/constraint_0_power_limit_uw"};

  for (const auto &path : rapl_paths) {
    long power_uw = this->read_sysfs_long(path);
    if (power_uw > 0) {
      return static_cast<double>(power_uw) / 1e6;
    }
  }
  return 0.0;
}

double Poirot::read_hwmon_power_w() {
  if (!this->cached_hwmon_power_path_.empty()) {
    long power_uw = this->read_sysfs_long(this->cached_hwmon_power_path_);
    if (power_uw > 0) {
      return static_cast<double>(power_uw) / 1e6;
    }
  }
  return 0.0;
}

void Poirot::iterate_hwmon_devices(
    std::function<bool(const std::string &, const std::string &)> callback) {
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
    std::string name = this->read_sysfs_string(base + "/name");
    if (name.empty()) {
      continue;
    }

    // Call callback with base path and device name
    // If callback returns true, stop iteration
    if (callback(base, name)) {
      break;
    }
  }

  closedir(hwmon_dir);
}

double Poirot::read_min_tdp_watts() {
  // Try RAPL constraint minimum power
  static const std::vector<std::string> min_power_paths = {
      "/sys/class/powercap/intel-rapl/intel-rapl:0/constraint_0_min_power_uw",
      "/sys/class/powercap/intel-rapl/intel-rapl:0/constraint_1_min_power_uw",
      "/sys/class/powercap/amd-rapl/amd-rapl:0/constraint_0_min_power_uw"};

  for (const auto &path : min_power_paths) {
    long power_uw = this->read_sysfs_long(path);
    if (power_uw > 0) {
      return static_cast<double>(power_uw) / 1e6;
    }
  }

  // Estimate from minimum CPU frequency
  long min_freq_khz = this->read_sysfs_long(
      "/sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_min_freq");
  if (min_freq_khz > 0 && this->system_info_.cpu_cores > 0) {
    double min_freq_ghz = static_cast<double>(min_freq_khz) / 1e6;
    double watts_per_ghz = this->read_watts_per_ghz();
    double min_tdp =
        this->system_info_.cpu_cores * min_freq_ghz * watts_per_ghz;
    if (min_tdp > 0) {
      return min_tdp;
    }
  }

  return FALLBACK_MIN_TDP_WATTS;
}

double Poirot::read_max_tdp_watts() {
  // Try RAPL max power constraint
  static const std::vector<std::string> max_power_paths = {
      "/sys/class/powercap/intel-rapl/intel-rapl:0/constraint_0_max_power_uw",
      "/sys/class/powercap/intel-rapl/intel-rapl:0/constraint_1_max_power_uw",
      "/sys/class/powercap/amd-rapl/amd-rapl:0/constraint_0_max_power_uw"};

  for (const auto &path : max_power_paths) {
    long power_uw = this->read_sysfs_long(path);
    if (power_uw > 0) {
      return static_cast<double>(power_uw) / 1e6;
    }
  }

  // Try hwmon power cap
  double max_tdp_w = 0.0;
  this->iterate_hwmon_devices(
      [this, &max_tdp_w](const std::string &base, const std::string &name) {
        if (name == "zenpower" || name == "amd_energy" || name == "coretemp" ||
            name == "k10temp") {
          long power_uw = this->read_sysfs_long(base + "/power1_cap_max");
          if (power_uw > 0) {
            max_tdp_w = static_cast<double>(power_uw) / 1e6;
            return true; // Stop iteration
          }
        }
        return false; // Continue iteration
      });

  if (max_tdp_w > 0.0) {
    return max_tdp_w;
  }

  // Estimate from max CPU frequency
  long max_freq_khz = this->read_sysfs_long(
      "/sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_max_freq");
  if (max_freq_khz > 0 && this->system_info_.cpu_cores > 0) {
    double max_freq_ghz = static_cast<double>(max_freq_khz) / 1e6;
    double watts_per_ghz = this->read_watts_per_ghz();
    double max_tdp =
        this->system_info_.cpu_cores * max_freq_ghz * watts_per_ghz;
    if (max_tdp > 0) {
      return max_tdp * 1.5; // Allow headroom for turbo boost
    }
  }

  return FALLBACK_MAX_TDP_WATTS;
}

double Poirot::read_idle_power_factor() {
  // Try to read actual idle power from RAPL or hwmon
  double idle_power_w = 0.0;
  double max_power_w = this->read_max_tdp_watts();

  // Read RAPL idle power (constraint_1 is often the idle/efficiency constraint)
  long idle_uw =
      this->read_sysfs_long("/sys/class/powercap/intel-rapl/intel-rapl:0/"
                            "constraint_1_power_limit_uw");
  if (idle_uw > 0) {
    idle_power_w = static_cast<double>(idle_uw) / 1e6;
  }

  // Try sustainable power as idle indicator
  if (idle_power_w == 0.0) {
    long sustainable_mw = this->read_sysfs_long(
        "/sys/class/thermal/thermal_zone0/sustainable_power");
    if (sustainable_mw > 0) {
      idle_power_w = static_cast<double>(sustainable_mw) / 1000.0;
    }
  }

  // Calculate ratio from min/max frequency
  if (idle_power_w == 0.0 || max_power_w <= 0.0) {
    long min_freq_khz = this->read_sysfs_long(
        "/sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_min_freq");
    long max_freq_khz = this->read_sysfs_long(
        "/sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_max_freq");
    if (min_freq_khz > 0 && max_freq_khz > 0) {
      // Power scales roughly with frequency squared (voltage scaling)
      double freq_ratio =
          static_cast<double>(min_freq_khz) / static_cast<double>(max_freq_khz);
      return freq_ratio * freq_ratio;
    }
  }

  if (idle_power_w > 0.0 && max_power_w > 0.0) {
    double factor = idle_power_w / max_power_w;
    // Clamp to reasonable range [0.05, 0.5]
    return std::max(0.05, std::min(0.5, factor));
  }

  return FALLBACK_IDLE_POWER_FACTOR;
}

double Poirot::read_watts_per_ghz() {
  // Try to calculate from actual power and frequency measurements
  double current_power_w = this->read_hwmon_power_w();
  if (current_power_w == 0.0) {
    current_power_w = this->read_rapl_power_limit_w();
  }

  long freq_khz = this->read_sysfs_long(
      "/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq");
  double current_freq_ghz = static_cast<double>(freq_khz) / 1e6;

  if (current_power_w > 0 && current_freq_ghz > 0 &&
      this->system_info_.cpu_cores > 0) {
    return current_power_w / (this->system_info_.cpu_cores * current_freq_ghz);
  }

  // Try from TDP and max frequency
  if (this->system_info_.cpu_tdp_watts > 0) {
    long max_freq_khz = this->read_sysfs_long(
        "/sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_max_freq");
    if (max_freq_khz > 0 && this->system_info_.cpu_cores > 0) {
      double max_freq_ghz = static_cast<double>(max_freq_khz) / 1e6;
      return this->system_info_.cpu_tdp_watts /
             (this->system_info_.cpu_cores * max_freq_ghz);
    }
  }

  // Fallback: typical value for modern CPUs (3-5 W/GHz/core)
  return FALLBACK_WATTS_PER_GHZ;
}

double Poirot::read_min_watts_per_ghz() {
  // Calculate minimum watts per GHz from system characteristics
  // Try to derive from minimum TDP and maximum frequency
  double min_tdp = this->read_min_tdp_watts();
  long max_freq_khz = this->read_sysfs_long(
      "/sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_max_freq");

  if (min_tdp > 0 && max_freq_khz > 0 && this->system_info_.cpu_cores > 0) {
    double max_freq_ghz = static_cast<double>(max_freq_khz) / 1e6;
    double min_watts_per_ghz =
        min_tdp / (this->system_info_.cpu_cores * max_freq_ghz);
    if (min_watts_per_ghz > 0) {
      return min_watts_per_ghz;
    }
  }

  // Try from idle power factor
  double watts_per_ghz = this->read_watts_per_ghz();
  double idle_factor = this->read_idle_power_factor();
  if (watts_per_ghz > 0 && idle_factor > 0) {
    return watts_per_ghz * idle_factor;
  }

  // Fallback: minimum reasonable value for modern CPUs
  return FALLBACK_MIN_WATTS_PER_GHZ;
}

double Poirot::read_max_watts_per_ghz() {
  // Calculate maximum watts per GHz from system characteristics
  // Try to derive from max TDP and base/min frequency (since turbo uses more)
  double max_tdp = this->read_max_tdp_watts();
  long base_freq_khz = this->read_sysfs_long(
      "/sys/devices/system/cpu/cpu0/cpufreq/base_frequency");

  // Fall back to scaling_max_freq if base_frequency not available
  if (base_freq_khz <= 0) {
    base_freq_khz = this->read_sysfs_long(
        "/sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq");
  }

  if (max_tdp > 0 && base_freq_khz > 0 && this->system_info_.cpu_cores > 0) {
    double base_freq_ghz = static_cast<double>(base_freq_khz) / 1e6;
    double max_watts_per_ghz =
        max_tdp / (this->system_info_.cpu_cores * base_freq_ghz);
    if (max_watts_per_ghz > 0) {
      return max_watts_per_ghz;
    }
  }

  // Try from current measurements with turbo headroom
  double watts_per_ghz = this->read_watts_per_ghz();
  if (watts_per_ghz > 0) {
    return watts_per_ghz * 2.0; // Allow 2x headroom for turbo
  }

  // Fallback: maximum reasonable value for modern CPUs
  return FALLBACK_MAX_WATTS_PER_GHZ;
}

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
  // CPU info: get real core count from /proc/cpuinfo
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

  // TDP detection: try multiple sources in order of reliability
  this->system_info_.cpu_tdp_watts = 0.0;
  this->system_info_.rapl_available = false;

  // Read RAPL max energy range for wraparound detection
  this->rapl_max_energy_uj_ = this->read_sysfs_double(
      "/sys/class/powercap/intel-rapl/intel-rapl:0/max_energy_range_uj");
  if (this->rapl_max_energy_uj_ == 0.0) {
    this->rapl_max_energy_uj_ = this->read_sysfs_double(
        "/sys/class/powercap/amd-rapl/amd-rapl:0/max_energy_range_uj");
  }

  // 1. Intel RAPL PL1 (long-term power limit = TDP)
  for (const auto &rapl_path : {"/sys/class/powercap/intel-rapl/intel-rapl:0/"
                                "constraint_0_power_limit_uw",
                                "/sys/class/powercap/intel-rapl/intel-rapl:0/"
                                "constraint_1_power_limit_uw"}) {
    long tdp_uw = this->read_sysfs_long(rapl_path);
    if (tdp_uw > 0) {
      this->system_info_.cpu_tdp_watts = static_cast<double>(tdp_uw) / 1e6;
      this->system_info_.cpu_tdp_watts_type =
          poirot_msgs::msg::SystemInfo::INTEL_RAPL_TDP_TYPE;
      this->system_info_.rapl_available = true;
      break;
    }
  }

  // 2. AMD RAPL power limit
  if (this->system_info_.cpu_tdp_watts == 0.0) {
    long tdp_uw = this->read_sysfs_long(
        "/sys/class/powercap/amd-rapl/amd-rapl:0/constraint_0_power_limit_uw");
    if (tdp_uw > 0) {
      this->system_info_.cpu_tdp_watts = static_cast<double>(tdp_uw) / 1e6;
      this->system_info_.cpu_tdp_watts_type =
          poirot_msgs::msg::SystemInfo::AMD_RAPL_TDP_TYPE;
      this->system_info_.rapl_available = true;
    }
  }

  // 3. Scan hwmon for power-related interfaces (zenpower, amd_energy, etc.)
  if (this->system_info_.cpu_tdp_watts == 0.0) {
    this->iterate_hwmon_devices([this](const std::string &base,
                                       const std::string &name) {
      // Check for CPU power monitoring drivers
      if (name == "zenpower" || name == "amd_energy" || name == "coretemp" ||
          name == "k10temp" || name.find("power") != std::string::npos) {

        static const std::vector<std::string> power_files = {
            "power1_cap", "power1_max", "power1_cap_max", "power1_rated_max"};

        for (const auto &power_file : power_files) {
          long power_uw = this->read_sysfs_long(base + "/" + power_file);
          if (power_uw > 0) {
            this->system_info_.cpu_tdp_watts =
                static_cast<double>(power_uw) / 1e6;
            this->system_info_.cpu_tdp_watts_type =
                poirot_msgs::msg::SystemInfo::HWMON_RAPL_TDP_TYPE;
            return true; // Stop iteration
          }
        }
      }
      return false; // Continue iteration
    });
  }

  // 4. Try thermal zone power budget
  if (this->system_info_.cpu_tdp_watts == 0.0) {

    DIR *thermal_dir = opendir("/sys/class/thermal");

    if (thermal_dir) {

      struct dirent *entry;
      // Look for cooling devices related to the CPU
      while ((entry = readdir(thermal_dir)) != nullptr) {

        if (strncmp(entry->d_name, "cooling_device", 14) == 0) {
          std::string base = std::string("/sys/class/thermal/") + entry->d_name;
          std::string type = this->read_sysfs_string(base + "/type");

          if (type.find("Processor") != std::string::npos ||
              type.find("processor") != std::string::npos) {

            long max_pstate = this->read_sysfs_long(base + "/max_state");
            if (max_pstate > 0) {
              // Try sustainable power from thermal zone
              long power_mw = this->read_sysfs_long(
                  "/sys/class/thermal/thermal_zone0/sustainable_power");

              if (power_mw > 0) {
                this->system_info_.cpu_tdp_watts =
                    static_cast<double>(power_mw) / 1000.0;
              }

              // Try cooling device power
              if (this->system_info_.cpu_tdp_watts == 0.0) {
                power_mw = this->read_sysfs_long(base + "/power");
                if (power_mw > 0) {
                  this->system_info_.cpu_tdp_watts =
                      static_cast<double>(power_mw) / 1000.0;
                }
              }

              // Try ACPI max power
              if (this->system_info_.cpu_tdp_watts == 0.0) {
                long power_uw = this->read_sysfs_long(
                    "/sys/class/powercap/intel-rapl/intel-rapl:0/"
                    "constraint_0_max_power_uw");
                if (power_uw > 0) {
                  this->system_info_.cpu_tdp_watts =
                      static_cast<double>(power_uw) / 1e6;
                }
              }

              // Derive from CPU frequency
              if (this->system_info_.cpu_tdp_watts == 0.0) {
                long max_freq_khz = this->read_sysfs_long(
                    "/sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_max_freq");
                if (max_freq_khz > 0 && this->system_info_.cpu_cores > 0) {
                  double max_freq_ghz = static_cast<double>(max_freq_khz) / 1e6;
                  double watts_per_ghz = this->read_watts_per_ghz();
                  double watts_per_core_at_max = max_freq_ghz * watts_per_ghz;
                  this->system_info_.cpu_tdp_watts =
                      this->system_info_.cpu_cores * watts_per_core_at_max;
                }
              }

              this->system_info_.cpu_tdp_watts_type =
                  poirot_msgs::msg::SystemInfo::THERMAL_POWER_TDP_TYPE;
              break;
            }
          }
        }
      }

      closedir(thermal_dir);
    }
  }

  // 5. Estimate from CPU frequency and core count (physics-based)
  if (this->system_info_.cpu_tdp_watts == 0.0) {
    long freq_khz = this->read_sysfs_long(
        "/sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_max_freq");
    double max_freq_ghz = static_cast<double>(freq_khz) / 1e6;

    if (max_freq_ghz > 0 && this->system_info_.cpu_cores > 0) {
      double power_factor = this->estimate_power_per_core_per_ghz();

      this->system_info_.cpu_tdp_watts =
          this->system_info_.cpu_cores * max_freq_ghz * power_factor;
      this->system_info_.cpu_tdp_watts_type =
          poirot_msgs::msg::SystemInfo::CPU_CORES_FREQUENCY_TYPE;
    }
  }

  // 6. Final fallback: estimate from core count alone
  if (this->system_info_.cpu_tdp_watts == 0.0) {
    // Derive watts_per_core from CPU model and system characteristics
    double watts_per_core = this->estimate_watts_per_core();

    this->system_info_.cpu_tdp_watts =
        this->system_info_.cpu_cores * watts_per_core;
    this->system_info_.cpu_tdp_watts_type =
        poirot_msgs::msg::SystemInfo::CPU_CORES_TYPE;
  }

  // Final sanity check: clamp TDP to system-derived bounds
  double min_tdp = this->read_min_tdp_watts();
  double max_tdp = this->read_max_tdp_watts();
  this->system_info_.cpu_tdp_watts =
      std::min(std::max(this->system_info_.cpu_tdp_watts, min_tdp), max_tdp);

  // Cache idle power factor for later use
  this->idle_power_factor_ = this->read_idle_power_factor();

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

  // Look for energy and power monitoring drivers
  this->iterate_hwmon_devices([this](const std::string &base,
                                     const std::string &name) {
    // Check for energy monitoring drivers
    bool is_energy_driver = (name == "zenpower" || name == "amd_energy" ||
                             name.find("energy") != std::string::npos);
    bool is_power_driver =
        (name == "zenpower" || name == "amd_energy" || name == "coretemp" ||
         name == "k10temp" || name.find("power") != std::string::npos);

    if (is_energy_driver && this->cached_hwmon_energy_path_.empty()) {
      for (int i = 1; i <= 10; ++i) {
        std::string energy_path =
            base + "/energy" + std::to_string(i) + "_input";
        if (this->read_sysfs_long(energy_path) >= 0 ||
            std::ifstream(energy_path).is_open()) {
          this->cached_hwmon_energy_path_ = energy_path;
          break;
        }
      }
    }

    if (is_power_driver && this->cached_hwmon_power_path_.empty()) {
      for (int i = 1; i <= 10; ++i) {
        std::string power_path = base + "/power" + std::to_string(i) + "_input";
        if (std::ifstream(power_path).is_open()) {
          this->cached_hwmon_power_path_ = power_path;
          break;
        }
      }
    }

    // Stop iteration if both paths are found
    return !this->cached_hwmon_energy_path_.empty() &&
           !this->cached_hwmon_power_path_.empty();
  });
}

double Poirot::estimate_power_per_core_per_ghz() {
  // Read actual power consumption from system and calculate real power per core
  // per GHz
  double current_power_w = this->read_hwmon_power_w();

  if (current_power_w == 0.0) {
    current_power_w = this->read_rapl_power_limit_w();
  }

  // Read current CPU frequency
  long freq_khz = this->read_sysfs_long(
      "/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq");
  double current_freq_ghz = static_cast<double>(freq_khz) / 1e6;

  // Calculate actual power per core per GHz if we have valid data
  if (current_power_w > 0 && current_freq_ghz > 0 &&
      this->system_info_.cpu_cores > 0) {
    double calculated =
        current_power_w / (this->system_info_.cpu_cores * current_freq_ghz);
    // Validate calculated value is in reasonable range
    double min_watts_per_ghz = this->read_min_watts_per_ghz();
    double max_watts_per_ghz = this->read_max_watts_per_ghz();
    if (calculated >= min_watts_per_ghz && calculated <= max_watts_per_ghz) {
      return calculated;
    }
  }

  // Try reading from DMI/SMBIOS for manufacturer specifications
  long max_speed_mhz =
      this->read_sysfs_long("/sys/class/dmi/id/processor_max_speed");
  if (max_speed_mhz > 0) {
    double max_freq_ghz = static_cast<double>(max_speed_mhz) / 1000.0;
    if (this->system_info_.cpu_tdp_watts > 0 &&
        this->system_info_.cpu_cores > 0) {
      double calculated = this->system_info_.cpu_tdp_watts /
                          (this->system_info_.cpu_cores * max_freq_ghz);
      return calculated;
    }
  }

  // Fallback: conservative default
  return FALLBACK_POWER_PER_CORE_PER_GHZ;
}

double Poirot::estimate_watts_per_core() {
  // Read actual watts per core from system measurements
  double total_power_w = this->read_hwmon_power_w();

  if (total_power_w == 0.0) {
    double battery_power = this->get_battery_power_w();
    if (battery_power > 0) {
      total_power_w = battery_power;
    }
  }

  if (total_power_w == 0.0) {
    total_power_w = this->read_rapl_power_limit_w();
  }

  if (total_power_w == 0.0) {
    long power_mw = this->read_sysfs_long(
        "/sys/class/thermal/thermal_zone0/sustainable_power");
    if (power_mw > 0) {
      total_power_w = static_cast<double>(power_mw) / 1000.0;
    }
  }

  if (total_power_w == 0.0 && this->system_info_.cpu_tdp_watts > 0) {
    total_power_w = this->system_info_.cpu_tdp_watts;
  }

  // Calculate watts per core from total power
  if (total_power_w > 0 && this->system_info_.cpu_cores > 0) {
    return total_power_w / this->system_info_.cpu_cores;
  }

  // Fallback: conservative default
  return FALLBACK_WATTS_PER_CORE;
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
  // This gives total CPU time across all cores and all processes
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

  if (total_energy_delta_uj <= 0.0) {
    return 0.0;
  }

  if (thread_cpu_delta_us <= 0.0) {
    // Thread consumed no CPU time, minimal energy for overhead
    constexpr double MIN_ATTRIBUTION_FACTOR = 0.001; // 0.1% minimum
    return total_energy_delta_uj * MIN_ATTRIBUTION_FACTOR;
  }

  double cpu_ratio = 0.0;

  // System-wide attribution is most accurate
  if (system_cpu_delta_us > 0.0) {
    // Thread's share of system-wide CPU = thread's share of package energy
    cpu_ratio = thread_cpu_delta_us / system_cpu_delta_us;

  } else if (process_cpu_delta_us > 0.0) {
    // Use process-level attribution (less accurate when other processes are
    // active)
    cpu_ratio = thread_cpu_delta_us / process_cpu_delta_us;

  } else {
    // No reference available, attribute all measured energy to thread
    return total_energy_delta_uj;
  }

  // Clamp ratio to [0, 1] to handle measurement anomalies
  cpu_ratio = std::max(0.0, std::min(1.0, cpu_ratio));

  // Calculate thread's energy consumption
  return total_energy_delta_uj * cpu_ratio;
}

long Poirot::read_thread_memory_kb() {
  std::string path = this->get_thread_status_path("status");
  std::ifstream file(path);
  if (!file.is_open()) {
    return 0;
  }

  std::string line;
  // Look for VmRSS line
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

  std::string path = this->get_thread_status_path("io");
  std::ifstream file(path);
  if (!file.is_open()) {
    return;
  }

  std::string line;
  while (std::getline(file, line)) {
    // Look for read_bytes and write_bytes lines
    if (line.compare(0, 11, "read_bytes:") == 0) {
      try {
        read_bytes = std::stol(line.substr(12));
      } catch (...) {
        read_bytes = 0;
      }

      // Look for write_bytes and write_bytes lines
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
  std::string path = this->get_thread_status_path("status");
  std::ifstream file(path);
  if (!file.is_open()) {
    return 0;
  }

  long total = 0;
  std::string line;
  while (std::getline(file, line)) {

    // Look for voluntary and nonvoluntary context switches
    if (line.compare(0, 24, "voluntary_ctxt_switches:") == 0) {
      std::istringstream iss(line.substr(24));
      long val = 0;
      iss >> val;
      total += val;

      // Look for nonvoluntary context switches
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
          // P = V * I
          // Both are in micro units -> result in pW, divide by 1e12 for W
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
  if (power_w <= 0) {
    power_w = this->read_hwmon_power_w();
  }

  // Estimate from TDP and CPU utilization
  if (power_w <= 0) {
    double cpu_pct = this->process_info_.cpu_percent;

    // Apply utilization factor with idle power baseline
    double idle_factor = this->idle_power_factor_;
    double util_factor = idle_factor + (1.0 - idle_factor) * (cpu_pct / 100.0);
    util_factor = std::max(idle_factor, std::min(1.0, util_factor));

    power_w = this->system_info_.cpu_tdp_watts * util_factor;
  }

  // Calculate energy: E = P * t (power in W, time in us -> energy in uJ)
  this->accumulated_energy_uj_ += power_w * elapsed_us;
  return this->accumulated_energy_uj_;
}

// ============================================================================
// System-Level Measurements
// ============================================================================
double Poirot::read_cpu_percent() {
  std::lock_guard<std::mutex> lock(this->cpu_read_mutex_);

  std::ifstream stat("/proc/self/stat");
  if (!stat.is_open()) {
    return 0.0;
  }

  std::string line;
  std::getline(stat, line);

  // Parse /proc/self/stat
  // Handle process names with spaces/parentheses
  // Format: pid (comm) state ppid pgrp session tty_nr tpgid flags minflt ...
  // Fields 14 (utime) and 15 (stime) are 0-indexed from the end of comm
  size_t comm_start = line.find('(');
  size_t comm_end = line.rfind(')');
  if (comm_start == std::string::npos || comm_end == std::string::npos) {
    return 0.0;
  }

  std::string after_comm = line.substr(comm_end + 2); // Skip the ") "
  std::istringstream iss(after_comm);
  std::string token;

  // Skip state(1), ppid(2), pgrp(3), session(4), tty_nr(5), tpgid(6),
  // flags(7), minflt(8), cminflt(9), majflt(10), cmajflt(11)
  // Then read utime(12) and stime(13) -> these are indices after comm
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

  this->process_info_.cpu_percent = this->read_cpu_percent();
  this->process_info_.mem_kb = this->read_thread_memory_kb();

  long io_read = 0;
  long io_write = 0;
  this->read_thread_io_bytes(io_read, io_write);

  this->process_info_.io_bytes = io_read + io_write;
  this->process_info_.threads = this->read_process_thread_count();
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

  // Prepare FunctionCall message
  poirot_msgs::msg::FunctionCall call;
  call.timestamp = rclcpp::Clock().now();
  call.data.wall_time_us =
      static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
                              end_time - ctx.start_time)
                              .count());
  call.data.cpu_time_us = thread_cpu_delta_us;
  call.data.mem_kb = end_memory_kb - ctx.start_memory_kb;
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
    stats.call = call;
  }

  if (this->verbose_.load()) {
    std::cout << "[PROFILE] " << ctx.function_name
              << " | Wall: " << call.data.wall_time_us << "us"
              << " | CPU: " << call.data.cpu_time_us << "us"
              << " | Mem: " << call.data.mem_kb << "KB"
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
  msg.timestamp = msg.function.call.timestamp;

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
