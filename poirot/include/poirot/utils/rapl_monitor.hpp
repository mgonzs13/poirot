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

#ifndef POIROT__UTILS__RAPL_MONITOR_HPP_
#define POIROT__UTILS__RAPL_MONITOR_HPP_

#include <mutex>
#include <string>

namespace poirot {
namespace utils {

constexpr char RAPL_PATH[] = "/sys/class/powercap/intel-rapl";

/**
 * @class EnergyMonitor
 * @brief Class for monitoring CPU energy consumption.
 *
 * Provides methods for reading energy from RAPL, hwmon, or estimating
 * from power measurements. Maintains state for tracking accumulated energy.
 */
class RaplMonitor {
public:
  /**
   * @brief Constructor.
   */
  RaplMonitor();

  /**
   * @brief Default destructor.
   */
  ~RaplMonitor() = default;

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
  /**
   * @brief Load RAPL package path.
   */
  void load_rapl_package_path();

  /**
   * @brief Initialize maximum RAPL energy value.
   */
  void initialize_rapl_max_energy();

  /// @brief Mutex for thread-safe RAPL readings
  mutable std::mutex rapl_mutex_;
  /// @brief RAPL package path
  std::string rapl_package_path_;
  /// @brief Accumulated energy in microjoules
  double accumulated_energy_uj_ = 0.0;
  /// @brief Last RAPL energy value (for delta calculation)
  double last_rapl_energy_uj_ = 0.0;
  /// @brief Maximum RAPL energy value before wrap-around
  double rapl_max_energy_uj_ = 0.0;
};

} // namespace utils
} // namespace poirot

#endif // POIROT__UTILS__RAPL_MONITOR_HPP_
