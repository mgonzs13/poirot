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

#ifndef POIROT__UTILS__NVML_LOADER_HPP_
#define POIROT__UTILS__NVML_LOADER_HPP_

#include <string>
#include <vector>

namespace poirot {
namespace utils {

/// @brief NVML device utilization rates (GPU and memory), in percent.
struct NvmlUtilization {
  unsigned int gpu = 0;
  unsigned int memory = 0;
};

/// @brief NVML device memory information in bytes.
struct NvmlMemory {
  unsigned long long total = 0;
  unsigned long long free = 0;
  unsigned long long used = 0;
};

/// @brief NVML per-process utilization sample.
struct NvmlProcessUtilizationSample {
  unsigned int pid = 0;
  unsigned long long time_stamp_us = 0;
  unsigned int sm_util = 0;
  unsigned int mem_util = 0;
  unsigned int enc_util = 0;
  unsigned int dec_util = 0;
};

/// @brief NVML process information (stable v2 layout).
struct NvmlProcessInfoV2 {
  unsigned int pid = 0;
  unsigned long long used_gpu_memory = 0;
  unsigned int gpu_instance_id = 0;
  unsigned int compute_instance_id = 0;
};

// ABI layout locks: the driver writes these structs directly. The v2 list API
// is used because drivers in the 535.43-535.98 window grew the v3 struct,
// which would overrun a 24-byte buffer.
static_assert(sizeof(NvmlUtilization) == 8, "NVML utilization layout changed");
static_assert(sizeof(NvmlMemory) == 24, "NVML memory layout changed");
static_assert(sizeof(NvmlProcessUtilizationSample) == 32,
              "NVML process utilization layout changed");
static_assert(sizeof(NvmlProcessInfoV2) == 24,
              "NVML process info layout changed");

/**
 * @class NvmlLoader
 * @brief Minimal runtime loader for the NVIDIA Management Library.
 *
 * Loads libnvidia-ml.so.1 with dlopen and resolves only the symbols POIROT
 * needs. No headers or build-time NVML dependency are required. The library is
 * intentionally never closed and nvmlShutdown() is never called because NVML
 * is process-wide and may be shared with other components.
 */
class NvmlLoader {
public:
  /**
   * @brief Constructor. Does not load the library.
   */
  NvmlLoader();

  /**
   * @brief Destructor. Does not close the library.
   */
  ~NvmlLoader();

  NvmlLoader(const NvmlLoader &) = delete;
  NvmlLoader &operator=(const NvmlLoader &) = delete;

  /**
   * @brief Load NVML and acquire device 0.
   * @return True when NVML is usable, false on any failure.
   */
  /// @note Call once before concurrent use; the method itself is not
  /// thread-safe.
  bool initialize();

  /**
   * @brief Check whether NVML is loaded and device 0 is usable.
   * @return True when available.
   */
  bool available() const { return this->available_; }

  /**
   * @brief Read the device name.
   * @return Device name, or an empty string when unavailable.
   */
  std::string name() const;

  /**
   * @brief Read device memory information.
   * @param memory Output memory information.
   * @return True on success.
   */
  bool read_memory(NvmlMemory &memory) const;

  /**
   * @brief Read the current device power draw in watts.
   * @param power_w Output power in watts.
   * @return True on success.
   */
  bool read_power_w(double &power_w) const;

  /**
   * @brief Read the device power management limit in watts.
   * @param power_limit_w Output limit in watts.
   * @return True on success.
   */
  bool read_power_limit_w(double &power_limit_w) const;

  /**
   * @brief Read the device GPU utilization in percent.
   * @param percent Output utilization percentage.
   * @return True on success.
   */
  bool read_device_utilization(double &percent) const;

  /**
   * @brief Read processes with GPU compute contexts.
   * @param processes Output process list.
   * @return True on success.
   */
  bool read_running_processes(std::vector<NvmlProcessInfoV2> &processes) const;

  /**
   * @brief Read per-process utilization samples.
   * @param samples Output sample list.
   * @return True on success.
   */
  bool read_process_utilization(
      std::vector<NvmlProcessUtilizationSample> &samples) const;

private:
  /// @brief Whether NVML was loaded and initialized successfully
  bool available_ = false;
  /// @brief dlopen handle (never closed)
  void *library_handle_ = nullptr;
  /// @brief NVML device handle for index 0
  void *device_ = nullptr;

  using InitFn = int (*)();
  using GetHandleFn = int (*)(unsigned int, void **);
  using GetNameFn = int (*)(void *, char *, unsigned int);
  using GetMemoryFn = int (*)(void *, NvmlMemory *);
  using GetPowerFn = int (*)(void *, unsigned int *);
  using GetLimitFn = int (*)(void *, unsigned int *);
  using GetUtilFn = int (*)(void *, NvmlUtilization *);
  using GetRunningFn = int (*)(void *, unsigned int *, NvmlProcessInfoV2 *);
  using GetProcUtilFn = int (*)(void *, NvmlProcessUtilizationSample *,
                                unsigned int *, unsigned long long);

  InitFn init_ = nullptr;
  GetHandleFn get_handle_ = nullptr;
  GetNameFn get_name_ = nullptr;
  GetMemoryFn get_memory_ = nullptr;
  GetPowerFn get_power_ = nullptr;
  GetLimitFn get_limit_ = nullptr;
  GetUtilFn get_util_ = nullptr;
  GetRunningFn get_running_ = nullptr;
  GetProcUtilFn get_proc_util_ = nullptr;
};

} // namespace utils
} // namespace poirot

#endif // POIROT__UTILS__NVML_LOADER_HPP_
