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

namespace poirot {
namespace utils {

/**
 * @class ThreadMetrics
 * @brief Utility class for reading thread-level metrics.
 *
 * Provides methods for measuring CPU time, memory, I/O, and context
 * switches for the current thread.
 */
class ThreadMetrics {
public:
  /**
   * @brief Default constructor.
   */
  ThreadMetrics() = default;

  /**
   * @brief Read thread CPU time in microseconds.
   * @return CPU time in microseconds.
   */
  long read_cpu_time_us();

  /**
   * @brief Read thread memory usage in KB.
   * @return Memory usage in KB.
   */
  long read_memory_kb();

  /**
   * @brief Read thread I/O bytes.
   * @param read_bytes Output parameter for read bytes.
   * @param write_bytes Output parameter for write bytes.
   */
  void read_io_bytes(long &read_bytes, long &write_bytes);

  /**
   * @brief Read thread context switches.
   * @return Number of voluntary context switches.
   */
  long read_ctx_switches();
};

} // namespace utils
} // namespace poirot

#endif // POIROT__UTILS__THREAD_METRICS_HPP_
