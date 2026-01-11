// Copyright 2026 Miguel Ángel González Santamarta
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

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <set>
#include <vector>

#include "poirot_msgs/msg/function_call.hpp"
#include "poirot_msgs/msg/function_stats.hpp"
#include "poirot_msgs/msg/profiling_data.hpp"

#include "poirot/poirot.hpp"

using namespace poirot;

Poirot::Poirot()
    : co2_manager_(), hwmon_scanner_(), power_estimator_(hwmon_scanner_),
      energy_monitor_(hwmon_scanner_), process_metrics_(), thread_metrics_() {
  this->node_ = rclcpp::Node::make_shared(
      "yasmin_" + utils::StringUtils::generate_uuid() + "_node");
  this->profiling_data_publisher_ =
      this->node_->create_publisher<poirot_msgs::msg::ProfilingData>(
          "poirot/data", rclcpp::QoS(10));
  this->auto_configure();
}

void Poirot::auto_configure() {
  // Download CO2 factors
  this->co2_manager_.download_factors();

  // Detect system information
  this->detect_system_info();

  // Search for hwmon paths
  this->hwmon_scanner_.search_paths();

  // Initialize energy monitor with RAPL max energy range
  this->energy_monitor_.initialize_rapl_max_energy();
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
        if (!this->system_info_.os_name.empty() &&
            this->system_info_.os_name.back() == '"') {
          this->system_info_.os_name.pop_back();
        }
        break;
      }
    }
  }

  // Configure power estimator with CPU info
  this->power_estimator_.set_cpu_cores(this->system_info_.cpu_cores);

  // RAPL detection
  this->system_info_.rapl_available = this->power_estimator_.rapl_available();

  // TDP detection
  // 1. Try Intel RAPL power limit (most accurate for Intel CPUs)
  double intel_rapl_power =
      this->power_estimator_.read_intel_rapl_power_limit_w();
  if (intel_rapl_power > 0) {
    this->system_info_.cpu_tdp_watts = intel_rapl_power;
    this->system_info_.cpu_tdp_watts_type =
        poirot_msgs::msg::SystemInfo::INTEL_RAPL_TDP_TYPE;
  }

  // 2. Try AMD RAPL power limit
  if (this->system_info_.cpu_tdp_watts == 0.0) {
    double amd_rapl_power =
        this->power_estimator_.read_amd_rapl_power_limit_w();
    if (amd_rapl_power > 0) {
      this->system_info_.cpu_tdp_watts = amd_rapl_power;
      this->system_info_.cpu_tdp_watts_type =
          poirot_msgs::msg::SystemInfo::AMD_RAPL_TDP_TYPE;
    }
  }

  // 3. Try hwmon power limit (TDP from hwmon drivers)
  if (this->system_info_.cpu_tdp_watts == 0.0) {
    double hwmon_tdp = this->power_estimator_.read_hwmon_tdp_watts();
    if (hwmon_tdp > 0) {
      this->system_info_.cpu_tdp_watts = hwmon_tdp;
      this->system_info_.cpu_tdp_watts_type =
          poirot_msgs::msg::SystemInfo::HWMON_RAPL_TDP_TYPE;
    }
  }

  // 4. Try thermal zone power budget
  if (this->system_info_.cpu_tdp_watts == 0.0) {
    double thermal_tdp = this->power_estimator_.read_thermal_tdp_watts();
    if (thermal_tdp > 0) {
      this->system_info_.cpu_tdp_watts = thermal_tdp;
      this->system_info_.cpu_tdp_watts_type =
          poirot_msgs::msg::SystemInfo::THERMAL_POWER_TDP_TYPE;
    }
  }

  // 5. Estimate from CPU frequency and core count (physics-based)
  if (this->system_info_.cpu_tdp_watts == 0.0) {
    double freq_tdp = this->power_estimator_.estimate_frequency_tdp_watts();
    if (freq_tdp > 0) {
      this->system_info_.cpu_tdp_watts = freq_tdp;
      this->system_info_.cpu_tdp_watts_type =
          poirot_msgs::msg::SystemInfo::CPU_CORES_FREQUENCY_TYPE;
    }
  }

  // 6. Final fallback: estimate from core count alone
  if (this->system_info_.cpu_tdp_watts == 0.0) {
    this->system_info_.cpu_tdp_watts =
        this->power_estimator_.estimate_cores_tdp_watts();
    this->system_info_.cpu_tdp_watts_type =
        poirot_msgs::msg::SystemInfo::CPU_CORES_TYPE;
  }

  // Final sanity check: clamp TDP to system-derived bounds
  double min_tdp = this->power_estimator_.read_min_tdp_watts();
  double max_tdp = this->power_estimator_.read_max_tdp_watts();
  this->system_info_.cpu_tdp_watts =
      std::min(std::max(this->system_info_.cpu_tdp_watts, min_tdp), max_tdp);

  // Update power estimator with TDP
  this->power_estimator_.set_cpu_tdp_watts(this->system_info_.cpu_tdp_watts);

  // Update energy monitor with TDP and idle factor
  this->energy_monitor_.set_cpu_tdp_watts(this->system_info_.cpu_tdp_watts);
  this->energy_monitor_.set_idle_power_factor(
      this->power_estimator_.read_idle_power_factor());

  // CO2 factor from timezone
  std::string timezone = this->co2_manager_.get_system_timezone();
  this->system_info_.country_code =
      this->co2_manager_.get_country_from_timezone(timezone);
  if (this->co2_manager_.factors_loaded()) {
    this->system_info_.co2_factor_kg_per_kwh =
        this->co2_manager_.get_factor_for_country(
            this->system_info_.country_code);
  } else {
    this->system_info_.co2_factor_kg_per_kwh =
        utils::DEFAULT_CO2_FACTOR_KG_PER_KWH;
  }
}

ThreadProfilingContext &Poirot::get_thread_context() {
  std::lock_guard<std::mutex> lock(this->contexts_mutex_);
  auto thread_id = std::this_thread::get_id();
  auto &ctx = this->thread_contexts_[thread_id];
  ctx.thread_id = thread_id;
  return ctx;
}

void Poirot::read_process_data() {
  this->process_info_.pid = getpid();
  this->process_info_.cpu_percent = this->process_metrics_.read_cpu_percent();
  this->process_info_.threads = this->process_metrics_.read_thread_count();
}

void Poirot::start_profiling(const std::string &function_name) {
  this->read_process_data();

  // Get thread-local context
  ThreadProfilingContext &ctx = this->get_thread_context();
  ctx.function_name = function_name;
  ctx.start_time = std::chrono::high_resolution_clock::now();
  ctx.start_cpu_time_us = this->thread_metrics_.read_cpu_time_us();
  ctx.start_process_cpu_time_us = this->process_metrics_.read_cpu_time_us();
  ctx.start_system_cpu_time_us =
      this->process_metrics_.read_total_cpu_time_us();
  ctx.start_memory_kb = this->thread_metrics_.read_memory_kb();
  this->thread_metrics_.read_io_bytes(ctx.start_io_read_bytes,
                                      ctx.start_io_write_bytes);
  ctx.start_ctx_switches = this->thread_metrics_.read_ctx_switches();
  ctx.start_energy_uj =
      this->energy_monitor_.read_energy_uj(this->process_info_.cpu_percent);
}

void Poirot::stop_profiling() {
  this->read_process_data();

  auto end_time = std::chrono::high_resolution_clock::now();
  double end_cpu_time_us = this->thread_metrics_.read_cpu_time_us();
  double end_process_cpu_time_us = this->process_metrics_.read_cpu_time_us();
  double end_system_cpu_time_us =
      this->process_metrics_.read_total_cpu_time_us();
  long end_memory_kb = this->thread_metrics_.read_memory_kb();
  long end_io_read_bytes = 0;
  long end_io_write_bytes = 0;
  this->thread_metrics_.read_io_bytes(end_io_read_bytes, end_io_write_bytes);
  long end_ctx_switches = this->thread_metrics_.read_ctx_switches();
  double end_energy_uj =
      this->energy_monitor_.read_energy_uj(this->process_info_.cpu_percent);

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
  double thread_energy_uj = this->energy_monitor_.calculate_thread_energy_uj(
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
  call.data.process_cpu_time_us = process_cpu_delta_us;
  call.data.system_cpu_time_us = system_cpu_delta_us;
  call.data.mem_kb = end_memory_kb - ctx.start_memory_kb;
  call.data.io_read_bytes = end_io_read_bytes - ctx.start_io_read_bytes;
  call.data.io_write_bytes = end_io_write_bytes - ctx.start_io_write_bytes;
  call.data.ctx_switches = end_ctx_switches - ctx.start_ctx_switches;
  call.data.energy_uj = thread_energy_uj;
  call.data.total_energy_uj = total_energy_delta_uj;

  // Calculate CO2 in micrograms
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

void Poirot::set_verbose(bool verbose) {
  get_instance().verbose_.store(verbose);
}

void Poirot::print_system_info() {
  const Poirot &instance = get_instance();
  std::cout << "\n=========================================="
            << "=========================\n";
  std::cout << "              SYSTEM INFORMATION\n";
  std::cout << "============================================"
            << "=======================\n";
  std::cout << "OS:       " << instance.system_info_.os_name << "\n";
  std::cout << "OS Version: " << instance.system_info_.os_version << "\n";
  std::cout << "Hostname: " << instance.system_info_.hostname << "\n";
  std::cout << "CPU:      " << instance.system_info_.cpu_model << "\n";
  std::cout << "Cores:    " << instance.system_info_.cpu_cores << "\n";
  std::cout << "Memory:   " << instance.system_info_.mem_total_kb / 1024
            << " MB\n";
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

void Poirot::publish_stats(const std::string &function_name) {
  std::shared_lock<std::shared_mutex> lock(this->statistics_mutex_);

  auto msg = poirot_msgs::msg::ProfilingData();
  msg.system_info = this->system_info_;
  msg.process_info = this->process_info_;
  msg.function = this->statistics_[function_name];
  msg.timestamp = msg.function.call.timestamp;

  this->profiling_data_publisher_->publish(msg);
}

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

Poirot &Poirot::get_instance() {
  static Poirot profiler;
  return profiler;
}
