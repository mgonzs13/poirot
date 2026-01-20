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

#ifndef POIROT__UTILS__THREAD_METRICS_HPP_
#define POIROT__UTILS__THREAD_METRICS_HPP_

#include <cstdint>

namespace poirot {
namespace utils {

/**
 * @struct ThreadIoBytes
 * @brief Structure to hold thread I/O byte counts.
 */
struct ThreadIoBytes {
  int64_t read_bytes = 0;
  int64_t write_bytes = 0;
};

/**
 * @class ThreadMetrics
 * @brief Utility class for reading thread-level metrics.
 *
 * Provides methods for measuring CPU time, memory, I/O, and context
 * switches for the current thread. All methods are thread-safe and
 * read data for the calling thread.
 */
class ThreadMetrics {
public:
  /**
   * @brief Default constructor.
   */
  ThreadMetrics() = default;

  /**
   * @brief Default destructor.
   */
  ~ThreadMetrics() = default;

  /**
   * @brief Read thread CPU time in microseconds.
   * @return CPU time in microseconds.
   */
  int64_t read_cpu_time_us() const;

  /**
   * @brief Read process CPU time in microseconds.
   * @return CPU time in microseconds.
   */
  int64_t read_process_cpu_time_us() const;

  /**
   * @brief Read thread memory usage in KB.
   * @return Memory usage in KB (VmRSS).
   */
  int64_t read_memory_kb() const;

  /**
   * @brief Read thread I/O bytes.
   * @return ThreadIoBytes structure with read and write byte counts.
   */
  ThreadIoBytes read_io_bytes() const;

  /**
   * @brief Read thread context switches (voluntary + non-voluntary).
   * @return Total number of context switches.
   */
  int64_t read_context_switches() const;

  /**
   * @brief Read the PID of the current process.
   * @return Process ID (PID).
   */
  int get_pid() const;

  /**
   * @brief Read number of threads in the current process.
   * @return Number of threads.
   */
  int read_num_threads() const;
};

} // namespace utils
} // namespace poirot

#endif // POIROT__UTILS__THREAD_METRICS_HPP_
