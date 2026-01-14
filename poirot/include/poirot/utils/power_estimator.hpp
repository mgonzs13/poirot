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

#ifndef POIROT__UTILS__POWER_ESTIMATOR_HPP_
#define POIROT__UTILS__POWER_ESTIMATOR_HPP_

#include <string>
#include <vector>

#include "poirot/utils/hwmon_scanner.hpp"

namespace poirot {
namespace utils {

/// @brief Fallback minimum TDP in watts if system detection fails
constexpr double FALLBACK_MIN_TDP_WATTS = 15.0;
/// @brief Fallback maximum TDP in watts if system detection fails
constexpr double FALLBACK_MAX_TDP_WATTS = 400.0;
/// @brief Fallback idle power factor if measurement fails
constexpr double FALLBACK_IDLE_POWER_FACTOR = 0.15;
/// @brief Fallback watts per core per GHz if calculation fails
constexpr double FALLBACK_WATTS_PER_GHZ = 4.0;
/// @brief Fallback minimum watts per GHz for validation
constexpr double FALLBACK_MIN_WATTS_PER_GHZ = 2.0;
/// @brief Fallback maximum watts per GHz for validation
constexpr double FALLBACK_MAX_WATTS_PER_GHZ = 20.0;
/// @brief Fallback power per core per GHz if all measurements fail
constexpr double FALLBACK_POWER_PER_CORE_PER_GHZ = 10.0;
/// @brief Fallback watts per core if all measurements fail
constexpr double FALLBACK_WATTS_PER_CORE = 12.0;

/// @brief CPU TDP detection type constants
enum class TdpType {
  INTEL_RAPL_TDP_TYPE = 1,
  AMD_RAPL_TDP_TYPE = 2,
  HWMON_RAPL_TDP_TYPE = 3,
  THERMAL_POWER_TDP_TYPE = 4,
  CPU_CORES_FREQUENCY_TYPE = 5,
  CPU_CORES_TYPE = 6
};

/**
 * @class PowerEstimator
 * @brief Class for estimating power consumption and TDP values.
 *
 * Provides methods for reading RAPL power limits, battery power,
 * and estimating various power-related metrics.
 */
class PowerEstimator {

public:
  /**
   * @brief Constructor.
   * @param hwmon_scanner Reference to HwmonScanner for power readings.
   */
  explicit PowerEstimator(HwmonScanner &hwmon_scanner);

  /**
   * @brief Set the number of CPU cores.
   * @param cores Number of CPU cores.
   */
  void set_cpu_cores(int cores) { this->cpu_cores_ = cores; }

  /**
   * @brief Set the CPU TDP in watts.
   * @param tdp TDP value in watts.
   */
  void set_cpu_tdp_watts(double tdp) { this->cpu_tdp_watts_ = tdp; }

  /**
   * @brief Check if RAPL is available.
   * @return True if RAPL is available, false otherwise.
   */
  bool rapl_available();

  /**
   * @brief Read CPU TDP in watts.
   * @return TDP in watts or 0.0 if not available.
   */
  std::pair<double, TdpType> read_tdp_watts();

  /**
   * @brief Read RAPL power limit in watts.
   * @return Power limit in watts or 0.0 if not available.
   */
  double read_rapl_power_limit_w();

  /**
   * @brief Read Intel RAPL power limit in watts.
   * @return Power limit in watts or 0.0 if not available.
   */
  double read_intel_rapl_power_limit_w();

  /**
   * @brief Read AMD RAPL power limit in watts.
   * @return Power limit in watts or 0.0 if not available.
   */
  double read_amd_rapl_power_limit_w();

  /**
   * @brief Read battery power in watts.
   * @return Battery power in watts or -1.0 if not available.
   */
  double read_battery_power_w();

  /**
   * @brief Read minimum TDP in watts.
   * @return Minimum TDP in watts.
   */
  double read_min_tdp_watts();

  /**
   * @brief Read maximum TDP in watts.
   * @return Maximum TDP in watts.
   */
  double read_max_tdp_watts();

  /**
   * @brief Read idle power factor.
   * @return Idle power factor (0.0 to 1.0).
   */
  double read_idle_power_factor();

  /**
   * @brief Read watts per GHz per core.
   * @return Watts per GHz per core.
   */
  double read_watts_per_ghz();

  /**
   * @brief Read minimum watts per GHz.
   * @return Minimum watts per GHz.
   */
  double read_min_watts_per_ghz();

  /**
   * @brief Read maximum watts per GHz.
   * @return Maximum watts per GHz.
   */
  double read_max_watts_per_ghz();

  /**
   * @brief Estimate power per core per GHz.
   * @return Estimated power per core per GHz.
   */
  double estimate_power_per_core_per_ghz();

  /**
   * @brief Estimate watts per core.
   * @return Estimated watts per core.
   */
  double estimate_watts_per_core();

  /**
   * @brief Read hwmon TDP from power limit files.
   * @return TDP in watts or 0.0 if not available.
   */
  double read_hwmon_tdp_watts();

  /**
   * @brief Read thermal zone power budget TDP.
   * @return TDP in watts or 0.0 if not available.
   */
  double read_thermal_tdp_watts();

  /**
   * @brief Estimate TDP from CPU frequency and core count.
   * @return TDP in watts or 0.0 if not available.
   */
  double estimate_frequency_tdp_watts();

  /**
   * @brief Estimate TDP from core count alone.
   * @return Estimated TDP in watts.
   */
  double estimate_cores_tdp_watts();

private:
  /// @brief Reference to HwmonScanner
  HwmonScanner &hwmon_scanner_;
  /// @brief Number of CPU cores
  int cpu_cores_ = 0;
  /// @brief CPU TDP in watts
  double cpu_tdp_watts_ = 0.0;
};

} // namespace utils
} // namespace poirot

#endif // POIROT__UTILS__POWER_HPP_
