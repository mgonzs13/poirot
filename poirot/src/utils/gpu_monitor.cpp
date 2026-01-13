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

#include <array>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>
#include <regex>
#include <sstream>
#include <unistd.h>

#include "poirot/utils/gpu_monitor.hpp"
#include "poirot/utils/sysfs_reader.hpp"

namespace poirot {
namespace utils {

namespace fs = std::filesystem;

GpuMonitor::GpuMonitor()
    : last_energy_read_time_(std::chrono::steady_clock::now()),
      last_process_energy_read_time_(std::chrono::steady_clock::now()) {}

bool GpuMonitor::initialize() {
  // Try to detect GPUs in order of preference
  if (this->detect_nvidia_gpu()) {
    this->gpu_vendor_ = GpuVendor::NVIDIA;
    return true;
  }

  if (this->detect_amd_gpu()) {
    this->gpu_vendor_ = GpuVendor::AMD;
    return true;
  }

  if (this->detect_intel_gpu()) {
    this->gpu_vendor_ = GpuVendor::INTEL;
    return true;
  }

  return false;
}

std::string GpuMonitor::exec_command(const std::string &cmd) {
  std::array<char, 128> buffer;
  std::string result;
  std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd.c_str(), "r"),
                                                pclose);
  if (!pipe) {
    return "";
  }

  while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
    result += buffer.data();
  }

  return result;
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
      this->gpu_info_.tdp_type = NVIDIA_SMI_TDP_TYPE;
      this->gpu_info_.power_monitoring = true;
    } catch (...) {
      this->gpu_info_.tdp_watts = FALLBACK_GPU_TDP_WATTS;
      this->gpu_info_.tdp_type = ESTIMATED_TDP_TYPE;
    }
  } else {
    this->gpu_info_.tdp_watts = FALLBACK_GPU_TDP_WATTS;
    this->gpu_info_.tdp_type = ESTIMATED_TDP_TYPE;
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
  // Look for AMD GPU in DRM subsystem
  const std::string drm_base = "/sys/class/drm";

  if (!fs::exists(drm_base)) {
    return false;
  }

  for (const auto &entry : fs::directory_iterator(drm_base)) {
    std::string card_path = entry.path().string();
    std::string card_name = entry.path().filename().string();

    // Look for card0, card1, etc.
    if (card_name.find("card") != 0 ||
        card_name.find("-") != std::string::npos) {
      continue;
    }

    // Check if it's an AMD GPU
    std::string device_path = card_path + "/device";
    std::string vendor_path = device_path + "/vendor";

    if (!fs::exists(vendor_path)) {
      continue;
    }

    std::string vendor_id = SysfsReader::read_string(vendor_path);
    // AMD vendor ID is 0x1002
    if (vendor_id.find("0x1002") == std::string::npos) {
      continue;
    }

    this->gpu_info_.vendor = "AMD";
    this->gpu_info_.available = true;

    // Try to get GPU name
    std::string product_path = device_path + "/product_name";
    if (fs::exists(product_path)) {
      this->gpu_info_.model = SysfsReader::read_string(product_path);
    } else {
      this->gpu_info_.model = "AMD GPU";
    }

    // Look for hwmon path for power/temp readings
    std::string hwmon_path = device_path + "/hwmon";
    if (fs::exists(hwmon_path)) {
      for (const auto &hwmon_entry : fs::directory_iterator(hwmon_path)) {
        std::string hwmon_base = hwmon_entry.path().string();

        // Power input (in microwatts)
        std::string power_path = hwmon_base + "/power1_average";
        if (fs::exists(power_path)) {
          this->amd_power_path_ = power_path;
          this->gpu_info_.power_monitoring = true;
        }

        // Temperature
        std::string temp_path = hwmon_base + "/temp1_input";
        if (fs::exists(temp_path)) {
          this->amd_temp_path_ = temp_path;
        }

        break; // Use first hwmon device
      }
    }

    // GPU busy percentage
    std::string gpu_busy = device_path + "/gpu_busy_percent";
    if (fs::exists(gpu_busy)) {
      this->amd_gpu_busy_path_ = gpu_busy;
    }

    // Memory busy percentage
    std::string mem_busy = device_path + "/mem_busy_percent";
    if (fs::exists(mem_busy)) {
      this->amd_mem_busy_path_ = mem_busy;
    }

    // VRAM usage
    std::string vram_used = device_path + "/mem_info_vram_used";
    if (fs::exists(vram_used)) {
      this->amd_vram_used_path_ = vram_used;
    }

    std::string vram_total = device_path + "/mem_info_vram_total";
    if (fs::exists(vram_total)) {
      this->amd_vram_total_path_ = vram_total;
      long total = SysfsReader::read_long(vram_total);
      if (total > 0) {
        this->gpu_info_.mem_total_kb = total / 1024;
      }
    }

    // Try to get TDP from power cap
    std::string power_cap = device_path + "/hwmon/hwmon*/power1_cap";
    if (this->gpu_info_.power_monitoring && !this->amd_power_path_.empty()) {
      // Derive power cap path from power average path
      std::string hwmon_base =
          this->amd_power_path_.substr(0, this->amd_power_path_.rfind('/'));
      std::string cap_path = hwmon_base + "/power1_cap";
      if (fs::exists(cap_path)) {
        long cap_uw = SysfsReader::read_long(cap_path);
        if (cap_uw > 0) {
          this->gpu_info_.tdp_watts = static_cast<double>(cap_uw) / 1e6;
          this->gpu_info_.tdp_type = SYSFS_TDP_TYPE;
        }
      }
    }

    if (this->gpu_info_.tdp_watts <= 0) {
      this->gpu_info_.tdp_watts = FALLBACK_GPU_TDP_WATTS;
      this->gpu_info_.tdp_type = ESTIMATED_TDP_TYPE;
    }

    return true;
  }

  return false;
}

bool GpuMonitor::detect_intel_gpu() {
  // Look for Intel GPU in DRM subsystem
  const std::string drm_base = "/sys/class/drm";

  if (!fs::exists(drm_base)) {
    return false;
  }

  for (const auto &entry : fs::directory_iterator(drm_base)) {
    std::string card_path = entry.path().string();
    std::string card_name = entry.path().filename().string();

    if (card_name.find("card") != 0 ||
        card_name.find("-") != std::string::npos) {
      continue;
    }

    std::string device_path = card_path + "/device";
    std::string vendor_path = device_path + "/vendor";

    if (!fs::exists(vendor_path)) {
      continue;
    }

    std::string vendor_id = SysfsReader::read_string(vendor_path);
    // Intel vendor ID is 0x8086
    if (vendor_id.find("0x8086") == std::string::npos) {
      continue;
    }

    this->gpu_info_.vendor = "Intel";
    this->gpu_info_.model = "Intel Integrated GPU";
    this->gpu_info_.available = true;
    this->gpu_info_.index = 0;

    // Intel GPUs typically don't have reliable power monitoring via sysfs
    // Use estimated TDP based on typical integrated GPU values
    this->gpu_info_.tdp_watts = 15.0; // Typical Intel iGPU TDP
    this->gpu_info_.tdp_type = ESTIMATED_TDP_TYPE;
    this->gpu_info_.power_monitoring = false;

    // Check for i915 frequency info
    std::string freq_path = "/sys/class/drm/" + card_name + "/gt_cur_freq_mhz";
    if (fs::exists(freq_path)) {
      this->intel_freq_path_ = freq_path;
    }

    return true;
  }

  return false;
}

GpuMetrics GpuMonitor::read_metrics() {
  switch (this->gpu_vendor_) {
  case GpuVendor::NVIDIA:
    return this->read_nvidia_metrics();
  case GpuVendor::AMD:
    return this->read_amd_metrics();
  case GpuVendor::INTEL:
    return this->read_intel_metrics();
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
    // Trim whitespace
    token.erase(0, token.find_first_not_of(" \t\n\r"));
    token.erase(token.find_last_not_of(" \t\n\r") + 1);
    values.push_back(token);
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

  // Read GPU utilization
  if (!this->amd_gpu_busy_path_.empty()) {
    long busy = SysfsReader::read_long(this->amd_gpu_busy_path_);
    if (busy >= 0) {
      metrics.utilization_percent = static_cast<double>(busy);
    }
  }

  // Read VRAM usage
  if (!this->amd_vram_used_path_.empty()) {
    long vram_used = SysfsReader::read_long(this->amd_vram_used_path_);
    if (vram_used > 0) {
      metrics.mem_used_kb = vram_used / 1024;
    }
  }

  // Read power (microwatts to watts)
  if (!this->amd_power_path_.empty()) {
    long power_uw = SysfsReader::read_long(this->amd_power_path_);
    if (power_uw > 0) {
      metrics.power_w = static_cast<double>(power_uw) / 1e6;
    }
  }

  // Calculate energy
  metrics.energy_uj =
      this->estimate_energy_uj(metrics.power_w, metrics.utilization_percent);

  return metrics;
}

GpuMetrics GpuMonitor::read_intel_metrics() {
  GpuMetrics metrics;

  // Intel iGPU has limited sysfs exposure
  // Read frequency if available
  if (!this->intel_freq_path_.empty()) {
    long freq_mhz = SysfsReader::read_long(this->intel_freq_path_);
    if (freq_mhz > 0) {
      // Estimate utilization based on frequency vs max
      // This is a rough approximation
      const double max_freq_mhz = 1500.0; // Typical max for Intel iGPU
      metrics.utilization_percent = std::min(
          100.0, (static_cast<double>(freq_mhz) / max_freq_mhz) * 100.0);
    }
  }

  // Estimate power for Intel iGPU
  double estimated_power =
      this->gpu_info_.tdp_watts *
      (this->idle_power_factor_ + (1.0 - this->idle_power_factor_) *
                                      (metrics.utilization_percent / 100.0));
  metrics.power_w = estimated_power;

  // Calculate energy
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
    double base_power = this->gpu_info_.tdp_watts * this->idle_power_factor_;
    double dynamic_power = this->gpu_info_.tdp_watts *
                           (1.0 - this->idle_power_factor_) *
                           (utilization / 100.0);
    double estimated_power_w = base_power + dynamic_power;
    double energy_delta_uj = estimated_power_w * elapsed_us;
    this->accumulated_energy_uj_ += energy_delta_uj;
  }

  return this->accumulated_energy_uj_;
}

double GpuMonitor::read_energy_uj() {
  GpuMetrics metrics = this->read_metrics();
  return metrics.energy_uj;
}

ProcessGpuMetrics GpuMonitor::read_process_metrics(pid_t pid) {
  if (pid == 0) {
    pid = getpid();
  }

  switch (this->gpu_vendor_) {
  case GpuVendor::NVIDIA:
    return this->read_nvidia_process_metrics(pid);
  case GpuVendor::AMD:
    return this->read_amd_process_metrics(pid);
  case GpuVendor::INTEL:
    return this->read_intel_process_metrics(pid);
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
    pid_str.erase(0, pid_str.find_first_not_of(" \t"));
    pid_str.erase(pid_str.find_last_not_of(" \t") + 1);
    mem_str.erase(0, mem_str.find_first_not_of(" \t"));
    mem_str.erase(mem_str.find_last_not_of(" \t") + 1);

    try {
      pid_t process_pid = static_cast<pid_t>(std::stoll(pid_str));
      if (process_pid == pid) {
        metrics.is_using_gpu = true;
        metrics.mem_used_kb = std::stoll(mem_str) * 1024; // MiB to KB
        break;
      }
    } catch (...) {
      continue;
    }
  }

  return metrics;
}

ProcessGpuMetrics GpuMonitor::read_amd_process_metrics(pid_t pid) {
  ProcessGpuMetrics metrics;

  // AMD GPUs expose per-process info via /proc/[pid]/fdinfo for DRM file
  // descriptors Look for DRM render node usage in /proc/[pid]/fd and check
  // fdinfo
  std::string fd_path = "/proc/" + std::to_string(pid) + "/fd";

  if (!fs::exists(fd_path)) {
    return metrics;
  }

  try {
    for (const auto &entry : fs::directory_iterator(fd_path)) {
      std::string link_path = entry.path().string();
      std::string target;

      try {
        target = fs::read_symlink(entry.path()).string();
      } catch (...) {
        continue;
      }

      // Check if this is a DRM render node (e.g., /dev/dri/renderD128)
      if (target.find("/dev/dri/render") == std::string::npos) {
        continue;
      }

      // Read fdinfo for this file descriptor
      std::string fdinfo_path = "/proc/" + std::to_string(pid) + "/fdinfo/" +
                                entry.path().filename().string();

      if (!fs::exists(fdinfo_path)) {
        continue;
      }

      std::ifstream fdinfo(fdinfo_path);
      if (!fdinfo.is_open()) {
        continue;
      }

      std::string fdinfo_line;
      while (std::getline(fdinfo, fdinfo_line)) {
        // Look for drm-memory-vram or amdgpu-vram fields
        if (fdinfo_line.find("drm-memory-vram:") != std::string::npos ||
            fdinfo_line.find("amdgpu-vram:") != std::string::npos) {
          metrics.is_using_gpu = true;
          // Extract memory value (in KiB)
          std::istringstream value_iss(fdinfo_line);
          std::string key;
          long value = 0;
          if (value_iss >> key >> value) {
            metrics.mem_used_kb += value;
          }
        }
      }
    }
  } catch (...) {
    // Permission or access error
  }

  return metrics;
}

ProcessGpuMetrics GpuMonitor::read_intel_process_metrics(pid_t pid) {
  ProcessGpuMetrics metrics;

  // Intel GPUs also expose per-process info via /proc/[pid]/fdinfo for i915/xe
  // DRM
  std::string fd_path = "/proc/" + std::to_string(pid) + "/fd";

  if (!fs::exists(fd_path)) {
    return metrics;
  }

  try {
    for (const auto &entry : fs::directory_iterator(fd_path)) {
      std::string target;

      try {
        target = fs::read_symlink(entry.path()).string();
      } catch (...) {
        continue;
      }

      // Check if this is a DRM card or render node
      if (target.find("/dev/dri/") == std::string::npos) {
        continue;
      }

      // Read fdinfo for this file descriptor
      std::string fdinfo_path = "/proc/" + std::to_string(pid) + "/fdinfo/" +
                                entry.path().filename().string();

      if (!fs::exists(fdinfo_path)) {
        continue;
      }

      std::ifstream fdinfo(fdinfo_path);
      if (!fdinfo.is_open()) {
        continue;
      }

      std::string fdinfo_line;
      while (std::getline(fdinfo, fdinfo_line)) {
        // Look for drm-memory-* or i915 specific fields
        if (fdinfo_line.find("drm-memory-") != std::string::npos) {
          metrics.is_using_gpu = true;
          // Extract memory value
          std::istringstream value_iss(fdinfo_line);
          std::string key;
          long value = 0;
          if (value_iss >> key >> value) {
            metrics.mem_used_kb += value;
          }
        }
        // Also check for drm-engine-render to detect GPU compute usage
        if (fdinfo_line.find("drm-engine-render:") != std::string::npos) {
          metrics.is_using_gpu = true;
        }
      }
    }
  } catch (...) {
    // Permission or access error
  }

  return metrics;
}

double GpuMonitor::read_process_energy_uj(pid_t pid) {
  if (pid == 0) {
    pid = getpid();
  }

  std::lock_guard<std::recursive_mutex> lock(this->energy_mutex_);

  auto now = std::chrono::steady_clock::now();

  if (this->last_process_energy_read_time_.time_since_epoch().count() == 0) {
    this->last_process_energy_read_time_ = now;
    return this->accumulated_process_energy_uj_;
  }

  double elapsed_us =
      static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
                              now - this->last_process_energy_read_time_)
                              .count());
  this->last_process_energy_read_time_ = now;

  if (elapsed_us <= 0) {
    return this->accumulated_process_energy_uj_;
  }

  // Check if process is actually using GPU
  ProcessGpuMetrics proc_metrics;
  switch (this->gpu_vendor_) {
  case GpuVendor::NVIDIA:
    proc_metrics = this->read_nvidia_process_metrics(pid);
    break;
  case GpuVendor::AMD:
    proc_metrics = this->read_amd_process_metrics(pid);
    break;
  case GpuVendor::INTEL:
    proc_metrics = this->read_intel_process_metrics(pid);
    break;
  default:
    return this->accumulated_process_energy_uj_;
  }

  // Only accumulate energy if process is actually using GPU
  if (!proc_metrics.is_using_gpu) {
    return this->accumulated_process_energy_uj_;
  }

  // Get system-wide GPU metrics for power/utilization
  GpuMetrics sys_metrics;
  switch (this->gpu_vendor_) {
  case GpuVendor::NVIDIA:
    sys_metrics = this->read_nvidia_metrics();
    break;
  case GpuVendor::AMD:
    sys_metrics = this->read_amd_metrics();
    break;
  case GpuVendor::INTEL:
    sys_metrics = this->read_intel_metrics();
    break;
  default:
    break;
  }

  // Estimate process's share of GPU power based on memory usage ratio
  double memory_ratio = 0.0;
  if (sys_metrics.mem_used_kb > 0 && proc_metrics.mem_used_kb > 0) {
    memory_ratio = static_cast<double>(proc_metrics.mem_used_kb) /
                   static_cast<double>(sys_metrics.mem_used_kb);
    // Clamp to [0, 1]
    memory_ratio = std::min(1.0, std::max(0.0, memory_ratio));
  } else if (proc_metrics.is_using_gpu) {
    // If we know process is using GPU but can't measure memory,
    // assume it's using some portion (this is a fallback)
    memory_ratio = 0.1; // Conservative estimate
  }

  // Calculate process's estimated power usage
  double process_power_w = sys_metrics.power_w * memory_ratio;
  if (process_power_w <= 0.0 && proc_metrics.is_using_gpu &&
      this->gpu_info_.tdp_watts > 0.0) {
    // Fallback: estimate based on TDP
    process_power_w = this->gpu_info_.tdp_watts * memory_ratio *
                      (sys_metrics.utilization_percent / 100.0);
  }

  // Accumulate energy
  double energy_delta_uj = process_power_w * elapsed_us;
  this->accumulated_process_energy_uj_ += energy_delta_uj;

  return this->accumulated_process_energy_uj_;
}

} // namespace utils
} // namespace poirot
