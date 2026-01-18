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

#ifndef POIROT__UTILS__ENERGY_MONITOR_HPP_
#define POIROT__UTILS__ENERGY_MONITOR_HPP_

#include <chrono>
#include <mutex>
#include <string>

#include "poirot/utils/hwmon_scanner.hpp"

namespace poirot {
namespace utils {

/**
 * @class EnergyMonitor
 * @brief Class for monitoring CPU energy consumption.
 *
 * Provides methods for reading energy from RAPL, hwmon, or estimating
 * from power measurements. Maintains state for tracking accumulated energy.
 */
class EnergyMonitor {
public:
  /**
   * @brief Constructor.
   * @param hwmon_scanner Reference to HwmonScanner for energy readings.
   */
  explicit EnergyMonitor(HwmonScanner &hwmon_scanner);

  /**
   * @brief Default destructor.
   */
  ~EnergyMonitor() = default;

  /**
   * @brief Set the CPU TDP in watts for estimation.
   * @param tdp TDP value in watts.
   */
  void set_cpu_tdp_watts(double tdp) { this->cpu_tdp_watts_ = tdp; }

  /**
   * @brief Set the idle power factor for estimation.
   * @param factor Idle power factor (0.0 to 1.0).
   */
  void set_idle_power_factor(double factor) {
    this->idle_power_factor_ = factor;
  }

  /**
   * @brief Initialize RAPL max energy range for wraparound detection.
   */
  void initialize_rapl_max_energy();

  /**
   * @brief Read accumulated CPU energy consumption in microjoules.
   * @return Accumulated energy consumption in microjoules.
   */
  double read_energy_uj();

  /**
   * @brief Calculate thread energy consumption using hierarchical attribution.
   *
   * This method calculates the energy attributed to a specific thread
   * based on its CPU time usage relative to the process and system.
   *
   * @param thread_cpu_delta_us Thread CPU time delta in microseconds.
   * @param process_cpu_delta_us Process CPU time delta in microseconds.
   * @param system_cpu_delta_us System CPU time delta in microseconds.
   * @param total_energy_delta_uj Total energy delta in microjoules.
   * @return Thread energy consumption in microjoules.
   */
  double calculate_thread_energy_uj(double thread_cpu_delta_us,
                                    double process_cpu_delta_us,
                                    double system_cpu_delta_us,
                                    double total_energy_delta_uj);

private:
  /// @brief Reference to HwmonScanner
  HwmonScanner &hwmon_scanner_;

  /// @brief Mutex for thread-safe energy readings
  mutable std::mutex energy_mutex_;
  /// @brief Accumulated energy in microjoules
  double accumulated_energy_uj_ = 0.0;
  /// @brief Last energy read time point
  std::chrono::steady_clock::time_point last_energy_read_time_;
  /// @brief Last RAPL energy value (for delta calculation)
  double last_rapl_energy_uj_ = 0.0;
  /// @brief Last hwmon energy value (for delta calculation)
  double last_hwmon_energy_uj_ = 0.0;
  /// @brief Maximum RAPL energy value before wrap-around
  double rapl_max_energy_uj_ = 0.0;

  /// @brief CPU TDP in watts for estimation
  double cpu_tdp_watts_ = 0.0;
  /// @brief Idle power factor for estimation (typical: 0.10-0.20)
  double idle_power_factor_ = 0.15;
};

} // namespace utils
} // namespace poirot

#endif // POIROT__UTILS__ENERGY_MONITOR_HPP_
