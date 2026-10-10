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

#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>
#include <regex>
#include <sstream>
#include <unordered_map>
#include <vector>

#include "poirot/utils/gpu_monitor.hpp"
#include "poirot/utils/string_utils.hpp"
#include "poirot/utils/sysfs_reader.hpp"

namespace poirot {
namespace utils {

namespace fs = std::filesystem;

double GpuMonitor::compute_process_share(
    pid_t pid, uint64_t now_us,
    const std::vector<NvmlProcessUtilizationSample> &samples,
    const std::vector<NvmlProcessInfoV2> &running) {
  // Keep only the newest fresh sample per PID so PIDs with multiple buffered
  // samples contribute exactly once to the normalizer.
  std::unordered_map<unsigned int, std::pair<unsigned long long, unsigned int>>
      newest_samples;

  for (const auto &sample : samples) {
    if (now_us < sample.time_stamp_us ||
        now_us - sample.time_stamp_us > FRESH_SAMPLE_WINDOW_US) {
      continue;
    }

    auto it = newest_samples.find(sample.pid);
    if (it == newest_samples.end() || sample.time_stamp_us > it->second.first) {
      newest_samples[sample.pid] = {sample.time_stamp_us, sample.sm_util};
    }
  }

  auto ours = newest_samples.find(static_cast<unsigned int>(pid));

  if (ours != newest_samples.end()) {
    double total_utilization = 0.0;
    for (const auto &entry : newest_samples) {
      total_utilization += static_cast<double>(entry.second.second);
    }

    if (total_utilization <= 0.0) {
      return 0.0;
    }

    double process_utilization = static_cast<double>(ours->second.second);
    return std::max(0.0,
                    std::min(1.0, process_utilization / total_utilization));
  }

  double process_memory = 0.0;
  double total_memory = 0.0;
  for (const auto &process : running) {
    total_memory += static_cast<double>(process.used_gpu_memory);
    if (static_cast<pid_t>(process.pid) == pid) {
      process_memory = static_cast<double>(process.used_gpu_memory);
    }
  }

  if (process_memory <= 0.0 || total_memory <= 0.0) {
    return 0.0;
  }

  return std::max(0.0, std::min(1.0, process_memory / total_memory));
}

double GpuMonitor::integrate_gap(double power_prev_w, double share_prev,
                                 double power_now_w, double share_now,
                                 double dt_us) {
  if (dt_us <= 0.0) {
    return 0.0;
  }

  return 0.5 * (power_prev_w * share_prev + power_now_w * share_now) * dt_us;
}

double GpuMonitor::select_power_w(bool power_valid, double power_w,
                                  double power_limit_w,
                                  double device_utilization_percent) {
  if (power_valid && power_w > 0.0) {
    return power_w;
  }

  if (power_limit_w > 0.0 && device_utilization_percent > 0.0) {
    return power_limit_w * device_utilization_percent / 100.0;
  }

  return 0.0;
}

GpuMonitor::GpuMonitor()
    : last_energy_read_time_(std::chrono::steady_clock::now()),
      last_process_energy_read_time_(std::chrono::steady_clock::now()) {
  this->initialize();
}

bool GpuMonitor::initialize() {
  // Try NVML first (no subprocess, per-process utilization available)
  if (this->detect_nvidia_gpu_nvml()) {
    this->gpu_vendor_ = GpuVendor::NVIDIA;
    return true;
  }

  // Fall back to the subprocess-based detection paths
  if (this->detect_nvidia_gpu()) {
    this->gpu_vendor_ = GpuVendor::NVIDIA;
    return true;
  }

  if (this->detect_amd_gpu()) {
    this->gpu_vendor_ = GpuVendor::AMD;
    return true;
  }

  return false;
}

std::string GpuMonitor::exec_command(const std::string &cmd) {
  std::array<char, 128> buffer;
  std::string result;
  std::unique_ptr<FILE, int (*)(FILE *)> pipe(popen(cmd.c_str(), "r"), pclose);
  if (!pipe) {
    return "";
  }

  while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
    result += buffer.data();
  }

  return result;
}

bool GpuMonitor::detect_nvidia_gpu_nvml() {
  if (!this->nvml_.initialize()) {
    return false;
  }

  std::string gpu_name = this->nvml_.name();
  if (gpu_name.empty()) {
    return false;
  }

  NvmlMemory memory;
  if (this->nvml_.read_memory(memory)) {
    this->gpu_info_.mem_total_kb = static_cast<int64_t>(memory.total / 1024);
  }

  double power_limit_w = 0.0;
  if (this->nvml_.read_power_limit_w(power_limit_w) && power_limit_w > 0.0) {
    this->gpu_info_.tdp_watts = power_limit_w;
    this->gpu_info_.tdp_type = GpuTdpType::NVIDIA_TDP_TYPE;
  }

  double power_w = 0.0;
  this->gpu_info_.power_monitoring =
      this->nvml_.read_power_w(power_w) && power_w > 0.0;

  this->gpu_info_.model = gpu_name;
  this->gpu_info_.vendor = "NVIDIA";
  this->gpu_info_.index = 0;
  this->gpu_info_.available = true;
  this->power_limit_w_ = power_limit_w;
  this->use_nvml_ = true;

  return true;
}

bool GpuMonitor::detect_nvidia_gpu() {
  // Check if nvidia-smi is available
  std::string output = this->exec_command("which nvidia-smi 2>/dev/null");
  if (output.empty()) {
    return false;
  }

  // Get GPU name
  output = this->exec_command(
      "nvidia-smi --query-gpu=name --format=csv,noheader,nounits 2>/dev/null");
  if (output.empty()) {
    return false;
  }

  this->gpu_info_.model = output;
  // Remove trailing newline
  if (!this->gpu_info_.model.empty() && this->gpu_info_.model.back() == '\n') {
    this->gpu_info_.model.pop_back();
  }
  this->gpu_info_.vendor = "NVIDIA";
  this->gpu_info_.index = 0;
  this->gpu_info_.available = true;

  // Get total memory
  output = this->exec_command(
      "nvidia-smi --query-gpu=memory.total --format=csv,noheader,nounits "
      "2>/dev/null");
  if (!output.empty()) {
    try {
      // nvidia-smi reports memory in MiB
      this->gpu_info_.mem_total_kb = std::stoll(output) * 1024;
    } catch (...) {
      this->gpu_info_.mem_total_kb = 0;
    }
  }

  // Get power limit (TDP)
  output = this->exec_command(
      "nvidia-smi --query-gpu=power.limit --format=csv,noheader,nounits "
      "2>/dev/null");
  if (!output.empty()) {
    try {
      this->gpu_info_.tdp_watts = std::stod(output);
      this->gpu_info_.tdp_type = GpuTdpType::NVIDIA_TDP_TYPE;
      this->gpu_info_.power_monitoring = true;
    } catch (...) {
      this->gpu_info_.tdp_watts = 0.0;
      this->gpu_info_.tdp_type = GpuTdpType::NO_TDP_TYPE;
    }
  } else {
    this->gpu_info_.tdp_watts = 0.0;
    this->gpu_info_.tdp_type = GpuTdpType::NO_TDP_TYPE;
  }

  // Check if power monitoring works
  output = this->exec_command(
      "nvidia-smi --query-gpu=power.draw --format=csv,noheader,nounits "
      "2>/dev/null");
  if (!output.empty() && output.find("[N/A]") == std::string::npos) {
    this->gpu_info_.power_monitoring = true;
  }

  return true;
}

bool GpuMonitor::detect_amd_gpu() {
  // Check if rocm-smi is available
  std::string output = this->exec_command("which rocm-smi 2>/dev/null");
  if (output.empty()) {
    return false;
  }

  // Get GPU model
  output = this->exec_command("rocm-smi --showproductname");
  if (output.empty()) {
    return false;
  }

  // Parse output
  std::istringstream iss(output);
  std::string line;
  while (std::getline(iss, line)) {
    if (line.find("Card Series") != std::string::npos &&
        line.find(":") != std::string::npos) {

      std::vector<std::string> parts;
      std::string token;
      std::istringstream line_iss(line);
      while (std::getline(line_iss, token, ':')) {
        parts.push_back(token);
      }

      if (parts.size() > 2) {
        this->gpu_info_.model = StringUtils::trim(parts[2]);
        break;
      }
    }
  }

  if (this->gpu_info_.model.empty()) {
    this->gpu_info_.model = "AMD GPU";
  }

  this->gpu_info_.vendor = "AMD";
  this->gpu_info_.index = 0;
  this->gpu_info_.available = true;

  // Get total memory
  output = this->exec_command("rocm-smi --showmeminfo vram");
  if (!output.empty()) {

    std::istringstream mem_iss(output);
    std::string mem_line;
    while (std::getline(mem_iss, mem_line)) {
      if (mem_line.find("VRAM Total Memory") != std::string::npos &&
          mem_line.find(":") != std::string::npos) {

        std::vector<std::string> parts;
        std::string token;
        std::istringstream line_iss(mem_line);
        while (std::getline(line_iss, token, ':')) {
          parts.push_back(token);
        }

        if (parts.size() > 2) {
          std::string mem_str = StringUtils::trim(parts[2]);

          try {
            long mem_bytes = std::stol(mem_str);
            this->gpu_info_.mem_total_kb = mem_bytes / 1024;
          } catch (...) {
            // ignore
          }
        }
        break;
      }
    }
  }

  // Get TDP
  output = this->exec_command("rocm-smi --showmaxpower");
  if (!output.empty()) {

    std::istringstream tdp_iss(output);
    std::string tdp_line;
    while (std::getline(tdp_iss, tdp_line)) {
      if (tdp_line.find("Package Power") != std::string::npos &&
          tdp_line.find(":") != std::string::npos) {

        std::vector<std::string> parts;
        std::string token;
        std::istringstream line_iss(tdp_line);
        while (std::getline(line_iss, token, ':')) {
          parts.push_back(token);
        }

        if (parts.size() > 2) {
          std::string tdp_str = StringUtils::trim(parts[2]);

          try {
            this->gpu_info_.tdp_watts = std::stod(tdp_str);
            this->gpu_info_.tdp_type = GpuTdpType::AMD_TDP_TYPE;
          } catch (...) {
            // ignore
          }
        }
        break;
      }
    }
  }

  // Check power monitoring
  output = this->exec_command("rocm-smi --showpower");
  if (!output.empty() && output.find("N/A") == std::string::npos) {
    this->gpu_info_.power_monitoring = true;
  }

  this->gpu_vendor_ = GpuVendor::AMD;
  return true;
}

GpuMetrics GpuMonitor::read_metrics() {
  if (!this->gpu_info_.available) {
    return GpuMetrics{};
  }

  if (this->use_nvml_) {
    return this->read_nvml_metrics();
  }

  switch (this->gpu_vendor_) {
  case GpuVendor::NVIDIA:
    return this->read_nvidia_metrics();
  case GpuVendor::AMD:
    return this->read_amd_metrics();
  default:
    return GpuMetrics{};
  }
}

GpuMetrics GpuMonitor::read_nvidia_metrics() {
  GpuMetrics metrics;

  // Query all metrics in a single nvidia-smi call for efficiency
  std::string output =
      this->exec_command("nvidia-smi "
                         "--query-gpu=utilization.gpu,memory.used,power.draw "
                         "--format=csv,noheader,nounits 2>/dev/null");

  if (output.empty()) {
    return metrics;
  }

  // Parse CSV output: utilization.gpu, memory.used, power.draw
  std::istringstream iss(output);
  std::string token;
  std::vector<std::string> values;

  while (std::getline(iss, token, ',')) {
    values.push_back(StringUtils::trim(token));
  }

  if (values.size() >= 3) {
    try {
      metrics.utilization_percent = std::stod(values[0]);
      metrics.mem_used_kb = std::stoll(values[1]) * 1024; // MiB to KB
      if (values[2] != "[N/A]") {
        metrics.power_w = std::stod(values[2]);
      }
    } catch (...) {
      // Parse error, return partial metrics
    }
  }

  // Calculate energy
  metrics.energy_uj =
      this->estimate_energy_uj(metrics.power_w, metrics.utilization_percent);

  return metrics;
}

GpuMetrics GpuMonitor::read_amd_metrics() {
  GpuMetrics metrics;

  try {
    // Read utilization
    std::string output = this->exec_command("rocm-smi --showuse");
    if (!output.empty()) {
      std::istringstream iss(output);
      std::string line;
      while (std::getline(iss, line)) {
        if (line.find("GPU use") != std::string::npos &&
            line.find(":") != std::string::npos) {

          std::vector<std::string> parts;
          std::string token;
          std::istringstream line_iss(line);
          while (std::getline(line_iss, token, ':')) {
            parts.push_back(token);
          }

          if (parts.size() > 2) {
            std::string use_str = StringUtils::trim(parts[2]);

            try {
              metrics.utilization_percent = std::stod(use_str);
            } catch (...) {
              // ignore
            }
          }
          break;
        }
      }
    }

    // Read memory used
    output = this->exec_command("rocm-smi --showmemuse");
    if (!output.empty()) {
      std::istringstream iss(output);
      std::string line;
      while (std::getline(iss, line)) {
        if (line.find("Memory Allocated") != std::string::npos &&
            line.find(":") != std::string::npos) {

          std::vector<std::string> parts;
          std::string token;
          std::istringstream line_iss(line);
          while (std::getline(line_iss, token, ':')) {
            parts.push_back(token);
          }

          if (parts.size() > 2) {
            std::string mem_str = StringUtils::trim(parts[2]);

            try {
              double mem_percent = std::stod(mem_str);
              metrics.mem_used_kb = static_cast<long>(
                  mem_percent / 100.0 * this->gpu_info_.mem_total_kb);
            } catch (...) {
              // ignore
            }
          }
          break;
        }
      }
    }

    // Read power
    output = this->exec_command("rocm-smi --showpower");
    if (!output.empty()) {
      std::istringstream iss(output);
      std::string line;
      while (std::getline(iss, line)) {
        if (line.find("Package Power") != std::string::npos &&
            line.find(":") != std::string::npos) {

          std::vector<std::string> parts;
          std::string token;
          std::istringstream line_iss(line);
          while (std::getline(line_iss, token, ':')) {
            parts.push_back(token);
          }

          if (parts.size() > 2) {
            std::string power_str = StringUtils::trim(parts[2]);

            try {
              metrics.power_w = std::stod(power_str);
            } catch (...) {
              // ignore
            }
          }
          break;
        }
      }
    }
  } catch (...) {
    // ignore
  }

  // Calculate energy (time-integrated)
  metrics.energy_uj =
      this->estimate_energy_uj(metrics.power_w, metrics.utilization_percent);

  return metrics;
}

double GpuMonitor::estimate_energy_uj(double power_w, double utilization) {
  std::lock_guard<std::recursive_mutex> lock(this->energy_mutex_);

  auto now = std::chrono::steady_clock::now();

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

  // If we have actual power reading, use it
  if (power_w > 0.0) {
    double energy_delta_uj = power_w * elapsed_us;
    this->accumulated_energy_uj_ += energy_delta_uj;
  } else if (this->gpu_info_.tdp_watts > 0.0) {
    // Estimate based on TDP and utilization
    double estimated_power_w =
        this->gpu_info_.tdp_watts * (utilization / 100.0);
    double energy_delta_uj = estimated_power_w * elapsed_us;
    this->accumulated_energy_uj_ += energy_delta_uj;
  }

  return this->accumulated_energy_uj_;
}

ProcessGpuMetrics GpuMonitor::read_process_metrics(pid_t pid) {
  if (pid == 0) {
    pid = getpid();
  }

  if (!this->gpu_info_.available) {
    return ProcessGpuMetrics{};
  }

  if (this->use_nvml_) {
    return this->read_nvml_process_metrics(pid);
  }

  switch (this->gpu_vendor_) {
  case GpuVendor::NVIDIA:
    return this->read_nvidia_process_metrics(pid);
  case GpuVendor::AMD:
    return this->read_amd_process_metrics(pid);
  default:
    return ProcessGpuMetrics{};
  }
}

bool GpuMonitor::is_process_using_gpu(pid_t pid) {
  if (pid == 0) {
    pid = getpid();
  }

  ProcessGpuMetrics metrics = this->read_process_metrics(pid);
  return metrics.is_using_gpu;
}

ProcessGpuMetrics GpuMonitor::read_nvidia_process_metrics(pid_t pid) {
  ProcessGpuMetrics metrics;

  // Query per-process GPU memory usage and type using nvidia-smi
  std::string cmd = "nvidia-smi --query-compute-apps=pid,used_memory "
                    "--format=csv,noheader,nounits 2>/dev/null";
  std::string output = this->exec_command(cmd);

  if (output.empty()) {
    return metrics;
  }

  // Parse CSV output line by line
  std::istringstream iss(output);
  std::string line;

  while (std::getline(iss, line)) {
    if (line.empty()) {
      continue;
    }

    // Parse: pid, used_memory
    std::istringstream line_iss(line);
    std::string pid_str, mem_str;

    if (!std::getline(line_iss, pid_str, ',') ||
        !std::getline(line_iss, mem_str, ',')) {
      continue;
    }

    // Trim whitespace
    pid_str = StringUtils::trim(pid_str);
    mem_str = StringUtils::trim(mem_str);

    try {
      pid_t process_pid = static_cast<pid_t>(std::stoll(pid_str));
      if (process_pid == pid) {
        metrics.is_using_gpu = true;
        metrics.mem_used_kb = std::stoll(mem_str) * 1024; // MiB to KB

        if (this->gpu_info_.mem_total_kb > 0) {
          metrics.estimated_utilization_percent =
              static_cast<double>(metrics.mem_used_kb) /
              static_cast<double>(this->gpu_info_.mem_total_kb) * 100.0;
        }

        break;
      }
    } catch (...) {
      continue;
    }
  }

  return metrics;
}

GpuMetrics GpuMonitor::read_nvml_metrics() {
  GpuMetrics metrics;

  double utilization_percent = 0.0;
  if (this->nvml_.read_device_utilization(utilization_percent)) {
    metrics.utilization_percent = utilization_percent;
  }

  NvmlMemory memory;
  if (this->nvml_.read_memory(memory)) {
    metrics.mem_used_kb = static_cast<int64_t>(memory.used / 1024);
  }

  double power_w = 0.0;
  if (this->nvml_.read_power_w(power_w)) {
    metrics.power_w = power_w;
  }

  // metrics.energy_uj stays 0 in the NVML path: it is not consumed
  // downstream; per-call energy uses read_process_energy_uj().
  return metrics;
}

ProcessGpuMetrics GpuMonitor::read_nvml_process_metrics(pid_t pid) {
  ProcessGpuMetrics metrics;

  std::vector<NvmlProcessInfoV2> running;
  this->nvml_.read_running_processes(running);
  for (const auto &process : running) {
    if (static_cast<pid_t>(process.pid) == pid) {
      metrics.is_using_gpu = true;
      metrics.mem_used_kb =
          static_cast<int64_t>(process.used_gpu_memory / 1024);
      break;
    }
  }

  std::vector<NvmlProcessUtilizationSample> samples;
  this->nvml_.read_process_utilization(samples);
  // NVML sample timestamps are realtime epoch microseconds, not steady clock.
  uint64_t realtime_us =
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::system_clock::now().time_since_epoch())
          .count();

  unsigned long long newest_timestamp = 0;
  for (const auto &sample : samples) {
    if (static_cast<pid_t>(sample.pid) != pid) {
      continue;
    }

    if (realtime_us < sample.time_stamp_us ||
        realtime_us - sample.time_stamp_us > FRESH_SAMPLE_WINDOW_US) {
      continue;
    }

    if (sample.time_stamp_us >= newest_timestamp) {
      newest_timestamp = sample.time_stamp_us;
      metrics.estimated_utilization_percent =
          static_cast<double>(sample.sm_util);
      metrics.is_using_gpu = true;
    }
  }

  if (newest_timestamp == 0 && metrics.is_using_gpu &&
      this->gpu_info_.mem_total_kb > 0) {
    metrics.estimated_utilization_percent =
        static_cast<double>(metrics.mem_used_kb) /
        static_cast<double>(this->gpu_info_.mem_total_kb) * 100.0;
  }

  return metrics;
}

ProcessGpuMetrics GpuMonitor::read_amd_process_metrics(pid_t pid) {
  ProcessGpuMetrics metrics;

  try {
    std::string output = this->exec_command("rocm-smi --showpids");
    if (!output.empty()) {
      std::istringstream iss(output);
      std::string line;
      while (std::getline(iss, line)) {
        if (line.empty()) {
          continue;
        }

        std::vector<std::string> parts;
        std::string token;
        std::istringstream line_iss(line);
        while (std::getline(line_iss, token, ' ')) {
          if (!token.empty()) {
            parts.push_back(token);
          }
        }

        if (parts.size() >= 4) {
          try {
            std::string pid_str = StringUtils::trim(parts[0]);
            pid_t proc_pid = static_cast<pid_t>(std::stol(pid_str));

            if (proc_pid == pid) {
              metrics.is_using_gpu = true;
              std::string mem_str = StringUtils::trim(parts[3]);
              long mem_bytes = std::stol(mem_str);
              metrics.mem_used_kb = mem_bytes / 1024;

              if (this->gpu_info_.mem_total_kb > 0) {
                metrics.estimated_utilization_percent =
                    static_cast<double>(metrics.mem_used_kb) /
                    static_cast<double>(this->gpu_info_.mem_total_kb) * 100.0;
              }

              break;
            }
          } catch (...) {
            continue;
          }
        }
      }
    }
  } catch (...) {
    // ignore
  }

  return metrics;
}

double GpuMonitor::read_process_energy_uj(pid_t pid) {
  if (pid == 0) {
    pid = getpid();
  }

  std::lock_guard<std::recursive_mutex> lock(this->energy_mutex_);

  if (!this->use_nvml_) {
    return this->read_process_energy_uj_legacy(pid);
  }

  uint64_t now_us = std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::steady_clock::now().time_since_epoch())
                        .count();
  uint64_t realtime_us =
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::system_clock::now().time_since_epoch())
          .count();

  if (this->last_process_read_us_ == 0) {
    // Initialize the integration state from a live sample so the first
    // interval is not half-charged by a zero trapezoid endpoint.
    double power_w = 0.0;
    bool power_valid = this->nvml_.read_power_w(power_w);
    double device_utilization = 0.0;
    this->nvml_.read_device_utilization(device_utilization);

    std::vector<NvmlProcessUtilizationSample> samples;
    std::vector<NvmlProcessInfoV2> running;
    this->nvml_.read_process_utilization(samples);
    this->nvml_.read_running_processes(running);

    this->last_power_w_ = select_power_w(
        power_valid, power_w, this->power_limit_w_, device_utilization);
    this->last_share_ =
        compute_process_share(pid, realtime_us, samples, running);
    this->last_process_read_us_ = now_us;
    return this->accumulated_process_energy_uj_;
  }

  double dt_us = static_cast<double>(now_us - this->last_process_read_us_);
  if (dt_us <= 0.0) {
    return this->accumulated_process_energy_uj_;
  }

  double power_w = 0.0;
  bool power_valid = this->nvml_.read_power_w(power_w);
  double device_utilization = 0.0;
  this->nvml_.read_device_utilization(device_utilization);
  double power = select_power_w(power_valid, power_w, this->power_limit_w_,
                                device_utilization);

  std::vector<NvmlProcessUtilizationSample> samples;
  std::vector<NvmlProcessInfoV2> running;
  this->nvml_.read_process_utilization(samples);
  this->nvml_.read_running_processes(running);
  double share = compute_process_share(pid, realtime_us, samples, running);

  this->accumulated_process_energy_uj_ += integrate_gap(
      this->last_power_w_, this->last_share_, power, share, dt_us);
  this->accumulated_share_us_ += share * dt_us;

  this->last_power_w_ = power;
  this->last_share_ = share;
  this->last_process_read_us_ = now_us;

  return this->accumulated_process_energy_uj_;
}

double GpuMonitor::read_process_energy_uj_legacy(pid_t pid) {
  auto now = std::chrono::steady_clock::now();

  // Initialize timestamp on first call
  if (this->last_process_energy_read_time_.time_since_epoch().count() == 0) {
    this->last_process_energy_read_time_ = now;
    return this->accumulated_process_energy_uj_;
  }

  double elapsed_us =
      static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
                              now - this->last_process_energy_read_time_)
                              .count());

  // Update timestamp regardless of GPU usage to avoid time gaps
  this->last_process_energy_read_time_ = now;

  if (elapsed_us <= 0) {
    return this->accumulated_process_energy_uj_;
  }

  // Get process-specific GPU metrics
  ProcessGpuMetrics proc_metrics;
  switch (this->gpu_vendor_) {
  case GpuVendor::NVIDIA:
    proc_metrics = this->read_nvidia_process_metrics(pid);
    break;
  case GpuVendor::AMD:
    proc_metrics = this->read_amd_process_metrics(pid);
    break;
  default:
    return this->accumulated_process_energy_uj_;
  }

  // Only accumulate energy if process is actually using GPU
  if (!proc_metrics.is_using_gpu) {
    return this->accumulated_process_energy_uj_;
  }

  // Accrue attributed utilization x time for the utilization metric.
  this->accumulated_share_us_ +=
      (proc_metrics.estimated_utilization_percent / 100.0) * elapsed_us;

  // Get system-wide GPU metrics for power/utilization
  GpuMetrics sys_metrics = this->read_metrics();

  // Calculate process's share of GPU power based on memory usage ratio
  double memory_ratio = 0.0;
  if (sys_metrics.mem_used_kb > 0 && proc_metrics.mem_used_kb > 0) {
    memory_ratio = static_cast<double>(proc_metrics.mem_used_kb) /
                   static_cast<double>(sys_metrics.mem_used_kb);
    // Clamp to [0, 1]
    memory_ratio = std::min(1.0, std::max(0.0, memory_ratio));
  }

  // Calculate process's estimated power usage
  double process_power_w = 0.0;
  if (sys_metrics.power_w > 0.0 && memory_ratio > 0.0) {
    // Use actual power reading multiplied by memory ratio
    process_power_w = sys_metrics.power_w * memory_ratio;
  } else if (this->gpu_info_.tdp_watts > 0.0) {
    // Estimate based on TDP, utilization, and memory ratio
    double utilization_factor = sys_metrics.utilization_percent / 100.0;
    process_power_w = this->gpu_info_.tdp_watts * utilization_factor;
  }

  // Calculate energy delta: E = P * t (power in watts, time in microseconds)
  // Result in microjoules: W * us = uJ
  if (process_power_w > 0.0) {
    double energy_delta_uj = process_power_w * elapsed_us;
    this->accumulated_process_energy_uj_ += energy_delta_uj;
  }

  return this->accumulated_process_energy_uj_;
}

double GpuMonitor::read_process_utilization_us(pid_t) {
  std::lock_guard<std::recursive_mutex> lock(this->energy_mutex_);
  return this->accumulated_share_us_;
}

} // namespace utils
} // namespace poirot
