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
#include <iomanip>
#include <vector>

#include "poirot_msgs/msg/cpu_info.hpp"
#include "poirot_msgs/msg/function_call.hpp"
#include "poirot_msgs/msg/function_stats.hpp"
#include "poirot_msgs/msg/profiling_data.hpp"

#include "poirot/poirot.hpp"
#include "poirot/utils/system_info_reader.hpp"

using namespace poirot;

Poirot::Poirot()
    : co2_manager_(), hwmon_scanner_(), power_estimator_(hwmon_scanner_),
      energy_monitor_(hwmon_scanner_), gpu_monitor_(), process_metrics_(),
      thread_metrics_() {

  // Create node options
  rclcpp::NodeOptions node_options;
  node_options.use_global_arguments(false);

  // Create POIROT node with unique name
  this->poirot_node_ = rclcpp::Node::make_shared(
      "poirot_" + utils::StringUtils::generate_uuid() + "_node", node_options);

  // Create profiling data publisher
  this->profiling_data_publisher_ =
      this->poirot_node_->create_publisher<poirot_msgs::msg::ProfilingData>(
          "poirot/data", rclcpp::QoS(100));
  this->auto_configure();
}

void Poirot::auto_configure() {
  // Detect system information
  this->detect_system_info();
}

void Poirot::detect_system_info() {
  // Use SystemInfoReader to read system information
  utils::SystemInfoReader system_reader;

  // CPU info
  auto cpu_info = system_reader.read_cpu_info();
  this->system_info_.cpu_info.model = cpu_info.model;
  this->system_info_.cpu_info.cores = cpu_info.cores;

  // Memory info
  auto mem_info = system_reader.read_memory_info();
  this->system_info_.mem_total_kb = mem_info.total_kb;

  // OS info
  auto os_info = system_reader.read_os_info();
  this->system_info_.os_name = os_info.name;
  this->system_info_.os_version = os_info.version;
  this->system_info_.hostname = os_info.hostname;

  // Configure power estimator with CPU info
  this->power_estimator_.set_cpu_cores(this->system_info_.cpu_info.cores);

  // RAPL detection
  this->system_info_.cpu_info.rapl_available =
      this->power_estimator_.rapl_available();

  // TDP detection
  auto [tdp_watts, tdp_type] = this->power_estimator_.read_tdp_watts();
  this->system_info_.cpu_info.tdp_watts = tdp_watts;

  if (tdp_type == utils::TdpType::INTEL_RAPL_TDP_TYPE) {
    this->system_info_.cpu_info.tdp_watts_type =
        poirot_msgs::msg::CpuInfo::INTEL_RAPL_TDP_TYPE;
  } else if (tdp_type == utils::TdpType::AMD_RAPL_TDP_TYPE) {
    this->system_info_.cpu_info.tdp_watts_type =
        poirot_msgs::msg::CpuInfo::AMD_RAPL_TDP_TYPE;
  } else if (tdp_type == utils::TdpType::HWMON_RAPL_TDP_TYPE) {
    this->system_info_.cpu_info.tdp_watts_type =
        poirot_msgs::msg::CpuInfo::HWMON_RAPL_TDP_TYPE;
  } else if (tdp_type == utils::TdpType::THERMAL_POWER_TDP_TYPE) {
    this->system_info_.cpu_info.tdp_watts_type =
        poirot_msgs::msg::CpuInfo::THERMAL_POWER_TDP_TYPE;
  } else if (tdp_type == utils::TdpType::CPU_CORES_FREQUENCY_TYPE) {
    this->system_info_.cpu_info.tdp_watts_type =
        poirot_msgs::msg::CpuInfo::CPU_CORES_FREQUENCY_TYPE;
  } else if (tdp_type == utils::TdpType::CPU_CORES_TYPE) {
    this->system_info_.cpu_info.tdp_watts_type =
        poirot_msgs::msg::CpuInfo::CPU_CORES_TYPE;
  }

  // Final sanity check: clamp TDP to system-derived bounds
  double min_tdp = this->power_estimator_.read_min_tdp_watts();
  double max_tdp = this->power_estimator_.read_max_tdp_watts();
  this->system_info_.cpu_info.tdp_watts = std::min(
      std::max(this->system_info_.cpu_info.tdp_watts, min_tdp), max_tdp);

  // Update power estimator with TDP
  this->power_estimator_.set_cpu_tdp_watts(
      this->system_info_.cpu_info.tdp_watts);
  // Update energy monitor with TDP and idle factor
  this->energy_monitor_.set_cpu_tdp_watts(
      this->system_info_.cpu_info.tdp_watts);

  // GPU detection and configuration
  if (this->gpu_monitor_.is_available()) {
    const auto &gpu_info = this->gpu_monitor_.get_gpu_info();
    this->system_info_.gpu_info.model = gpu_info.model;
    this->system_info_.gpu_info.vendor = gpu_info.vendor;
    this->system_info_.gpu_info.index = gpu_info.index;
    this->system_info_.gpu_info.mem_total_kb = gpu_info.mem_total_kb;
    this->system_info_.gpu_info.tdp_watts = gpu_info.tdp_watts;

    if (gpu_info.tdp_type == utils::GpuTdpType::NVIDIA_SMI_TDP_TYPE) {
      this->system_info_.gpu_info.tdp_watts_type =
          poirot_msgs::msg::GpuInfo::NVIDIA_SMI_TDP_TYPE;
    } else if (gpu_info.tdp_type == utils::GpuTdpType::AMD_ROCM_TDP_TYPE) {
      this->system_info_.gpu_info.tdp_watts_type =
          poirot_msgs::msg::GpuInfo::AMD_ROCM_TDP_TYPE;
    } else if (gpu_info.tdp_type == utils::GpuTdpType::SYSFS_TDP_TYPE) {
      this->system_info_.gpu_info.tdp_watts_type =
          poirot_msgs::msg::GpuInfo::SYSFS_TDP_TYPE;
    } else if (gpu_info.tdp_type == utils::GpuTdpType::ESTIMATED_TDP_TYPE) {
      this->system_info_.gpu_info.tdp_watts_type =
          poirot_msgs::msg::GpuInfo::ESTIMATED_TDP_TYPE;
    }

    this->system_info_.gpu_info.available = true;
    this->system_info_.gpu_info.power_monitoring = gpu_info.power_monitoring;
  } else {
    this->system_info_.gpu_info.available = false;
    this->system_info_.gpu_info.power_monitoring = false;
  }

  // CO2 factor from timezone
  auto co2_info = this->co2_manager_.get_co2_info();
  this->system_info_.co2_info.country_code = co2_info.country_code;
  this->system_info_.co2_info.co2_factor_loaded = co2_info.co2_factor_loaded;
  this->system_info_.co2_info.co2_factor_kg_per_kwh =
      co2_info.co2_factor_kg_per_kwh;
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
  this->process_info_.threads = this->process_metrics_.read_thread_count();
}

void Poirot::start_profiling(const std::string &function_name,
                             const std::string &file, int line) {
  this->read_process_data();

  // Get thread-local context
  ThreadProfilingContext &ctx = this->get_thread_context();
  ctx.function_name = function_name;
  ctx.file = file;
  ctx.line = line;
  ctx.start_time = std::chrono::steady_clock::now();
  ctx.start_cpu_time_us = this->thread_metrics_.read_cpu_time_us();
  ctx.start_process_cpu_time_us = this->process_metrics_.read_cpu_time_us();
  ctx.start_mem_kb = this->thread_metrics_.read_memory_kb();

  auto io_bytes = this->thread_metrics_.read_io_bytes();
  ctx.start_io_read_bytes = io_bytes.read_bytes;
  ctx.start_io_write_bytes = io_bytes.write_bytes;

  ctx.start_context_switches = this->thread_metrics_.read_context_switches();
  std::tie(ctx.start_cpu_energy_uj, std::ignore) =
      this->energy_monitor_.read_energy_uj();

  // Capture GPU energy start if GPU is available
  if (this->gpu_monitor_.is_available()) {
    // Read per-process GPU metrics instead of system-wide
    auto process_metrics = this->gpu_monitor_.read_process_metrics();
    ctx.start_gpu_utilization_percent =
        process_metrics.is_using_gpu
            ? process_metrics.estimated_utilization_percent
            : 0.0;
    ctx.start_gpu_mem_kb = process_metrics.mem_used_kb;
    ctx.start_gpu_energy_uj = this->gpu_monitor_.read_process_energy_uj();

  } else {
    ctx.start_gpu_utilization_percent = 0.0;
    ctx.start_gpu_mem_kb = 0;
    ctx.start_gpu_energy_uj = 0.0;
  }
}

void Poirot::stop_profiling() {
  this->read_process_data();

  auto end_time = std::chrono::steady_clock::now();
  int64_t end_cpu_time_us = this->thread_metrics_.read_cpu_time_us();
  int64_t end_process_cpu_time_us = this->process_metrics_.read_cpu_time_us();
  int64_t end_mem_kb = this->thread_metrics_.read_memory_kb();

  auto end_io_bytes = this->thread_metrics_.read_io_bytes();
  int64_t end_io_read_bytes = end_io_bytes.read_bytes;
  int64_t end_io_write_bytes = end_io_bytes.write_bytes;

  int64_t end_context_switches = this->thread_metrics_.read_context_switches();

  // Read GPU metrics if available
  double end_gpu_utilization_percent = 0.0;
  int64_t end_gpu_mem_kb = 0;
  double end_gpu_energy_uj = 0.0;
  if (this->gpu_monitor_.is_available()) {
    // Read per-process GPU metrics instead of system-wide
    auto process_metrics = this->gpu_monitor_.read_process_metrics();
    end_gpu_utilization_percent =
        process_metrics.is_using_gpu
            ? process_metrics.estimated_utilization_percent
            : 0.0;
    end_gpu_mem_kb = process_metrics.mem_used_kb;
    end_gpu_energy_uj = this->gpu_monitor_.read_process_energy_uj();
  }

  // Get thread-local context
  ThreadProfilingContext &ctx = this->get_thread_context();

  // Calculate deltas
  double wall_time_delta_us =
      static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
                              end_time - ctx.start_time)
                              .count());
  double thread_cpu_delta_us =
      static_cast<double>(end_cpu_time_us - ctx.start_cpu_time_us);
  double process_cpu_delta_us = static_cast<double>(
      end_process_cpu_time_us - ctx.start_process_cpu_time_us);
  double system_cpu_delta_us =
      wall_time_delta_us *
      static_cast<double>(this->process_metrics_.get_num_cpus());

  double gpu_energy_delta_uj = end_gpu_energy_uj - ctx.start_gpu_energy_uj;

  // Calculate thread-level CPU energy using hierarchical CPU time attribution
  double end_cpu_energy_uj = 0.0;
  poirot::utils::EnergyType energy_type;
  std::tie(end_cpu_energy_uj, energy_type) =
      this->energy_monitor_.read_energy_uj(wall_time_delta_us);
  double cpu_total_energy_delta_uj =
      end_cpu_energy_uj - ctx.start_cpu_energy_uj;

  double thread_cpu_energy_uj =
      this->energy_monitor_.calculate_thread_energy_uj(
          thread_cpu_delta_us, process_cpu_delta_us, system_cpu_delta_us,
          cpu_total_energy_delta_uj);

  // Calculate total energy (CPU + GPU)
  double total_energy_uj = thread_cpu_energy_uj + gpu_energy_delta_uj;

  // Prepare FunctionCall message
  poirot_msgs::msg::FunctionCall call;
  call.timestamp = rclcpp::Clock().now();
  call.data.wall_time_us = static_cast<int64_t>(wall_time_delta_us);
  call.data.cpu_time_us = static_cast<int64_t>(thread_cpu_delta_us);
  call.data.process_cpu_time_us = static_cast<int64_t>(process_cpu_delta_us);
  call.data.system_cpu_time_us = static_cast<int64_t>(system_cpu_delta_us);
  call.data.mem_kb = end_mem_kb - ctx.start_mem_kb;
  call.data.io_read_bytes = end_io_read_bytes - ctx.start_io_read_bytes;
  call.data.io_write_bytes = end_io_write_bytes - ctx.start_io_write_bytes;
  call.data.ctx_switches = end_context_switches - ctx.start_context_switches;
  call.data.cpu_energy_uj = thread_cpu_energy_uj;
  call.data.cpu_total_energy_uj = cpu_total_energy_delta_uj;

  // GPU data (calculate deltas correctly: end - start)
  call.data.gpu_utilization_percent =
      end_gpu_utilization_percent - ctx.start_gpu_utilization_percent;
  call.data.gpu_mem_kb = end_gpu_mem_kb - ctx.start_gpu_mem_kb;
  call.data.gpu_energy_uj = gpu_energy_delta_uj;

  // Total energy (CPU + GPU)
  if (energy_type == poirot::utils::EnergyType::ENERGY_TYPE_RAPL_INTEL) {
    call.data.cpu_energy_type = poirot_msgs::msg::Data::ENERGY_TYPE_RAPL_INTEL;
  } else if (energy_type == poirot::utils::EnergyType::ENERGY_TYPE_RAPL_AMD) {
    call.data.cpu_energy_type = poirot_msgs::msg::Data::ENERGY_TYPE_RAPL_AMD;
  } else if (energy_type == poirot::utils::EnergyType::ENERGY_TYPE_HWMON) {
    call.data.cpu_energy_type = poirot_msgs::msg::Data::ENERGY_TYPE_HWMON;
  } else if (energy_type ==
             poirot::utils::EnergyType::ENERGY_TYPE_HWMON_ESTIMATED) {
    call.data.cpu_energy_type =
        poirot_msgs::msg::Data::ENERGY_TYPE_HWMON_ESTIMATED;
  } else if (energy_type == poirot::utils::EnergyType::ENERGY_TYPE_ESTIMATED) {
    call.data.cpu_energy_type = poirot_msgs::msg::Data::ENERGY_TYPE_ESTIMATED;
  }

  call.data.total_energy_uj = total_energy_uj;

  // Calculate CO2 in micrograms based on total energy
  constexpr double UJ_TO_KWH = 1.0 / 1e6 / 3600.0 / 1000.0;
  constexpr double KG_TO_UG = 1e9;
  double energy_kwh = call.data.total_energy_uj * UJ_TO_KWH;
  call.data.co2_ug =
      energy_kwh * this->system_info_.co2_info.co2_factor_kg_per_kwh * KG_TO_UG;

  {
    std::unique_lock<std::shared_mutex> lock(this->statistics_mutex_);

    auto &stats = this->statistics_[ctx.function_name];
    stats.name = ctx.function_name;
    stats.file = ctx.file;
    stats.line = ctx.line;
    stats.call_count++;
    stats.call = call;
  }

  if (this->verbose_.load()) {
    fprintf(
        stderr,
        "[PROFILE] %s | Wall: %ldus | CPU: %ldus | Mem: %ldKB | IO R/W: "
        "%ld/%ldB | CtxSw: %ld | CPU Energy: %.2fuJ | GPU Energy: %.2fuJ | "
        "Total Energy: %.2fuJ | CO2: %.2fug\n",
        ctx.function_name.c_str(), static_cast<long>(call.data.wall_time_us),
        static_cast<long>(call.data.cpu_time_us),
        static_cast<long>(call.data.mem_kb),
        static_cast<long>(call.data.io_read_bytes),
        static_cast<long>(call.data.io_write_bytes),
        static_cast<long>(call.data.ctx_switches), call.data.cpu_energy_uj,
        call.data.gpu_energy_uj, call.data.total_energy_uj, call.data.co2_ug);
  }

  this->publish_stats(ctx.function_name);
}

void Poirot::set_verbose(bool verbose) {
  get_instance().verbose_.store(verbose);
}

void Poirot::print_system_info() {
  const Poirot &instance = get_instance();
  fprintf(
      stderr,
      "\n================================================================\n");
  fprintf(stderr, "              SYSTEM INFORMATION\n");
  fprintf(stderr,
          "================================================================\n");
  fprintf(stderr, "OS:       %s\n", instance.system_info_.os_name.c_str());
  fprintf(stderr, "OS Version: %s\n", instance.system_info_.os_version.c_str());
  fprintf(stderr, "Hostname: %s\n", instance.system_info_.hostname.c_str());
  fprintf(stderr, "CPU:      %s\n",
          instance.system_info_.cpu_info.model.c_str());
  fprintf(stderr, "Cores:    %d\n", instance.system_info_.cpu_info.cores);
  fprintf(stderr, "Memory:   %ld MB\n",
          instance.system_info_.mem_total_kb / 1024);
  fprintf(stderr, "RAPL:     %s\n",
          instance.system_info_.cpu_info.rapl_available ? "Yes" : "No");
  fprintf(stderr, "CPU TDP:  %f W\n", instance.system_info_.cpu_info.tdp_watts);
  fprintf(stderr, "TDP Type: %d\n",
          static_cast<int>(instance.system_info_.cpu_info.tdp_watts_type));
  fprintf(stderr,
          "----------------------------------------------------------------\n");
  fprintf(stderr, "GPU:      %s\n",
          instance.system_info_.gpu_info.available
              ? instance.system_info_.gpu_info.model.c_str()
              : "Not available");

  if (instance.system_info_.gpu_info.available) {
    fprintf(stderr, "GPU Vendor: %s\n",
            instance.system_info_.gpu_info.vendor.c_str());
    fprintf(stderr, "GPU Memory: %ld MB\n",
            instance.system_info_.gpu_info.mem_total_kb / 1024);
    fprintf(stderr, "GPU TDP:  %f W\n",
            instance.system_info_.gpu_info.tdp_watts);
    fprintf(stderr, "GPU Power Mon: %s\n",
            instance.system_info_.gpu_info.power_monitoring ? "Yes" : "No");
  }

  fprintf(stderr,
          "----------------------------------------------------------------\n");
  fprintf(stderr, "Country:  %s\n",
          instance.system_info_.co2_info.country_code.c_str());
  fprintf(stderr, "CO2 Factor Loaded: %s\n",
          instance.system_info_.co2_info.co2_factor_loaded ? "Yes" : "No");
  fprintf(stderr, "CO2:      %f kg/kWh\n",
          instance.system_info_.co2_info.co2_factor_kg_per_kwh);
  fprintf(stderr,
          "================================================================\n");
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
  profiler_.start_profiling(extract_function_name(func), file, line);
}

ScopedPoirot::~ScopedPoirot() { profiler_.stop_profiling(); }

Poirot &Poirot::get_instance() {
  static Poirot profiler;
  return profiler;
}
