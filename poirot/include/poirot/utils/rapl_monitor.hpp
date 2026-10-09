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

#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>

namespace poirot {
namespace utils {

constexpr char RAPL_PATH[] = "/sys/class/powercap/intel-rapl";

/**
 * @struct EnergySnapshot
 * @brief Point-in-time CPU energy and system busy-time reading.
 */
struct EnergySnapshot {
  /// @brief Accumulated package energy in microjoules (wrap-handled).
  double energy_uj = 0.0;
  /// @brief Cumulative system busy CPU time in microseconds.
  double busy_us = 0.0;
  /// @brief False when RAPL is unavailable or unreadable.
  bool valid = false;
};

/**
 * @struct RateWindow
 * @brief Lazy windowed estimator of energy per system busy microsecond.
 *
 * The window is opened on first use and refreshed only when it has been
 * open for at least the configured window duration, so it can be driven
 * by call-boundary reads without a dedicated sampler thread.
 */
struct RateWindow {
  /// @brief Energy per system busy microsecond in uJ/us.
  double rate_uj_per_busy_us = 0.0;
  /// @brief Accumulated energy at window start in uJ.
  double start_energy_uj = 0.0;
  /// @brief System busy time at window start in us.
  double start_busy_us = 0.0;
  /// @brief Window start time in us (0 = uninitialized).
  int64_t start_time_us = 0;

  /**
   * @brief Refresh the rate when the window has been open for window_us.
   *
   * Samples with no positive energy and busy-time deltas are discarded,
   * but the window is always reset once it elapses so idle periods
   * cannot poison the rate.
   *
   * @param now_us Current steady-clock time in microseconds.
   * @param energy_uj Current accumulated energy in microjoules.
   * @param busy_us Current cumulative system busy time in microseconds.
   * @param window_us Window duration in microseconds.
   */
  void update(int64_t now_us, double energy_uj, double busy_us,
              int64_t window_us);
};

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

  /// @brief Minimum interval for the measured attribution path (20 ms).
  static constexpr double MIN_INTERVAL_US = 20000.0;

  /**
   * @brief Attribute a measured package-energy delta to a thread.
   *
   * Uses the interval path when a measurable energy delta and at least
   * MIN_INTERVAL_US of system busy time are available; otherwise falls
   * back to the measured rate. Returns 0 when no usable data exists.
   *
   * @param delta_energy_uj Package energy delta over the call interval.
   * @param delta_busy_us System busy CPU time delta over the call interval.
   * @param thread_cpu_delta_us Thread CPU time delta over the call interval.
   * @param rate_uj_per_busy_us Fallback energy rate in uJ per busy us.
   * @return Attributed thread energy in microjoules.
   */
  static double attribute_energy(double delta_energy_uj, double delta_busy_us,
                                 double thread_cpu_delta_us,
                                 double rate_uj_per_busy_us);

  /**
   * @brief Parse the aggregate `cpu` line of /proc/stat into busy time.
   *
   * Busy time is user + nice + system + irq + softirq + steal; guest
   * time is already included in user/nice and must not be added again.
   *
   * @param line The first line of /proc/stat (starting with "cpu").
   * @param clk_tck Ticks per second (_SC_CLK_TCK); falls back to 100 if <= 0.
   * @return Busy CPU time in microseconds, or 0.0 on parse failure.
   */
  static double parse_proc_stat_busy_us(const std::string &line, long clk_tck);

  /**
   * @brief Read a combined CPU energy and system busy-time snapshot.
   *
   * Also refreshes the lazy fallback rate window when it has elapsed.
   *
   * @return Snapshot; `valid` is false when RAPL is unreadable.
   */
  EnergySnapshot read_snapshot();

  /**
   * @brief Calculate the CPU energy attributed to a profiled call.
   *
   * @param start Snapshot taken at function start.
   * @param end Snapshot taken at function end.
   * @param thread_cpu_delta_us Thread CPU time consumed by the call.
   * @return Attributed energy in microjoules (never negative).
   */
  double calculate_call_energy_uj(const EnergySnapshot &start,
                                  const EnergySnapshot &end,
                                  double thread_cpu_delta_us);

protected:
  /**
   * @brief Load RAPL package path.
   */
  void load_rapl_package_path();

  /**
   * @brief Initialize maximum RAPL energy value.
   */
  void initialize_rapl_max_energy();

private:
  /// @brief Lazy fallback rate window.
  RateWindow rate_window_;
  /// @brief Rate window duration in microseconds.
  static constexpr int64_t RATE_WINDOW_US = 250000;

  /**
   * @brief Read accumulated RAPL energy; caller must hold rapl_mutex_.
   * @return Accumulated energy in uJ, or -1.0 when unavailable.
   */
  double read_energy_uj_locked();

  /**
   * @brief Read system busy time from /proc/stat; caller holds rapl_mutex_.
   * @return Cumulative busy time in microseconds, or 0.0 on failure.
   */
  double read_system_busy_us_locked() const;

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
