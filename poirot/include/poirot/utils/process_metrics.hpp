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

#ifndef POIROT__UTILS__PROCESS_METRICS_HPP_
#define POIROT__UTILS__PROCESS_METRICS_HPP_

#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>
#include <vector>

namespace poirot {
namespace utils {

/**
 * @class ProcessMetrics
 * @brief Class for reading process-level metrics with state tracking.
 *
 * Provides methods for measuring process CPU time, system CPU time,
 * and thread count. Maintains state for CPU percentage calculation.
 */
class ProcessMetrics {
public:
  /**
   * @brief Default constructor.
   */
  ProcessMetrics();

  /**
   * @brief Destructor.
   */
  ~ProcessMetrics();

  /**
   * @brief Read process CPU time in microseconds.
   * @return CPU time in microseconds.
   */
  double read_cpu_time_us();

  /**
   * @brief Read total available CPU time in microseconds.
   *
   * Returns wall clock time × number of CPU cores, representing
   * the total CPU time available to the system. Uses high-resolution
   * monotonic clock for nanosecond precision.
   * @return Total available CPU time in microseconds.
   */
  double read_total_cpu_time_us();

  /**
   * @brief Read process thread count.
   * @return Number of threads.
   */
  int read_thread_count();

  /**
   * @brief Read process CPU usage percentage.
   *
   * This method requires state tracking and should be called on an instance.
   * @return CPU usage percentage.
   */
  double read_cpu_percent();

private:
  /// @brief Previous process CPU time
  std::atomic<unsigned long long> prev_process_cpu_{0};
  /// @brief Previous CPU read time point
  std::chrono::high_resolution_clock::time_point prev_cpu_read_time_;
  /// @brief Mutex for CPU read time updates
  mutable std::mutex cpu_read_mutex_;

  /// @brief Number of CPU cores
  int num_cpus_ = 1;
};

} // namespace utils
} // namespace poirot

#endif // POIROT__UTILS__PROCESS_METRICS_HPP_
