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

#ifndef POIROT__UTILS__GPU_MONITOR_HPP_
#define POIROT__UTILS__GPU_MONITOR_HPP_

#include <sys/types.h>

#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

namespace poirot {
namespace utils {

/// @brief Fallback GPU TDP in watts if detection fails
constexpr double FALLBACK_GPU_TDP_WATTS = 150.0;

/// @brief GPU TDP detection type constants
enum class GpuTdpType {
  NO_TDP_TYPE = 0,
  NVIDIA_SMI_TDP_TYPE = 1,
  AMD_ROCM_TDP_TYPE = 2,
  SYSFS_TDP_TYPE = 3
};

/**
 * @struct GpuMetrics
 * @brief Structure to hold current GPU metrics.
 */
struct GpuMetrics {
  double utilization_percent = 0.0;
  int64_t mem_used_kb = 0;
  double power_w = 0.0;
  double energy_uj = 0.0;
};

/**
 * @struct ProcessGpuMetrics
 * @brief Structure to hold per-process GPU metrics.
 */
struct ProcessGpuMetrics {
  bool is_using_gpu = false;
  int64_t mem_used_kb = 0;
  double estimated_utilization_percent = 0.0;
  double estimated_power_w = 0.0;
};

/**
 * @struct GpuInfo
 * @brief Structure to hold static GPU information.
 */
struct GpuInfo {
  std::string model;
  std::string vendor;
  int index = 0;
  int64_t mem_total_kb = 0;
  double tdp_watts = 0.0;
  GpuTdpType tdp_type = GpuTdpType::NO_TDP_TYPE;
  bool available = false;
  bool power_monitoring = false;
};

/**
 * @class GpuMonitor
 * @brief Class for monitoring GPU metrics and energy consumption.
 *
 * Supports NVIDIA GPUs via nvidia-smi and AMD GPUs via ROCm/sysfs.
 * Provides methods for reading GPU utilization, memory usage, power, and
 * energy.
 */
class GpuMonitor {
public:
  /**
   * @brief Default constructor.
   */
  GpuMonitor();

  /**
   * @brief Default destructor.
   */
  ~GpuMonitor() = default;

  /**
   * @brief Check if GPU monitoring is available.
   * @return True if GPU is available for monitoring.
   */
  bool is_available() const { return this->gpu_info_.available; }

  /**
   * @brief Get GPU information.
   * @return Const reference to GpuInfo structure.
   */
  const GpuInfo &get_gpu_info() const { return this->gpu_info_; }

  /**
   * @brief Read current GPU metrics.
   * @return GpuMetrics structure with current values.
   */
  GpuMetrics read_metrics();

  /**
   * @brief Read per-process GPU metrics for a specific PID.
   * @param pid Process ID to query (default: current process).
   * @return ProcessGpuMetrics structure with current values.
   */
  ProcessGpuMetrics read_process_metrics(pid_t pid = 0);

  /**
   * @brief Check if a specific process is using the GPU.
   * @param pid Process ID to check (default: current process).
   * @return True if the process is using GPU compute.
   */
  bool is_process_using_gpu(pid_t pid = 0);

  /**
   * @brief Read per-process accumulated GPU energy in microjoules.
   *
   * This method calculates the energy consumption attributed to a specific
   * process based on its GPU memory usage ratio compared to system-wide
   * GPU usage. Uses time-based integration of power measurements.
   *
   * @param pid Process ID to query (default: current process).
   * @return Accumulated energy attributed to the process in microjoules.
   */
  double read_process_energy_uj(pid_t pid = 0);

protected:
  /**
   * @brief Initialize GPU monitoring and detect available GPUs.
   * @return True if GPU was detected and initialized.
   */
  bool initialize();

private:
  /**
   * @brief Detect NVIDIA GPU using nvidia-smi.
   * @return True if NVIDIA GPU was detected.
   */
  bool detect_nvidia_gpu();

  /**
   * @brief Detect AMD GPU using ROCm or sysfs.
   * @return True if AMD GPU was detected.
   */
  bool detect_amd_gpu();

  /**
   * @brief Read NVIDIA GPU metrics via nvidia-smi.
   * @return GpuMetrics structure with current values.
   */
  GpuMetrics read_nvidia_metrics();

  /**
   * @brief Read AMD GPU metrics via sysfs.
   * @return GpuMetrics structure with current values.
   */
  GpuMetrics read_amd_metrics();

  /**
   * @brief Read NVIDIA per-process GPU metrics via nvidia-smi.
   * @param pid Process ID to query.
   * @return ProcessGpuMetrics structure with current values.
   */
  ProcessGpuMetrics read_nvidia_process_metrics(pid_t pid);

  /**
   * @brief Read AMD per-process GPU metrics via sysfs/fdinfo.
   * @param pid Process ID to query.
   * @return ProcessGpuMetrics structure with current values.
   */
  ProcessGpuMetrics read_amd_process_metrics(pid_t pid);

  /**
   * @brief Execute a command and return its output.
   * @param cmd Command to execute.
   * @return Command output as string.
   */
  std::string exec_command(const std::string &cmd);

  /**
   * @brief Estimate energy based on power and elapsed time.
   * @param power_w Current power in watts.
   * @param utilization GPU utilization percentage.
   * @return Energy delta in microjoules.
   */
  double estimate_energy_uj(double power_w, double utilization);

  /// @brief GPU information structure
  GpuInfo gpu_info_;

  /// @brief GPU vendor type for optimized metric reading
  enum class GpuVendor {
    NONE,
    NVIDIA,
    AMD,
  } gpu_vendor_ = GpuVendor::NONE;

  /// @brief Mutex for thread-safe energy readings
  mutable std::recursive_mutex energy_mutex_;
  /// @brief Accumulated GPU energy in microjoules (system-wide)
  double accumulated_energy_uj_ = 0.0;
  /// @brief Accumulated per-process GPU energy in microjoules
  double accumulated_process_energy_uj_ = 0.0;
  /// @brief Last energy read time point
  std::chrono::steady_clock::time_point last_energy_read_time_;
  /// @brief Last per-process energy read time point
  std::chrono::steady_clock::time_point last_process_energy_read_time_;

  /// @brief AMD GPU sysfs paths
  std::string amd_gpu_busy_path_;
  std::string amd_mem_busy_path_;
  std::string amd_power_path_;
  std::string amd_temp_path_;
  std::string amd_vram_used_path_;
  std::string amd_vram_total_path_;
};

} // namespace utils
} // namespace poirot

#endif // POIROT__UTILS__GPU_MONITOR_HPP_
