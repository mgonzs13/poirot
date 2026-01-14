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

#ifndef POIROT__UTILS__SYSFS_READER_HPP_
#define POIROT__UTILS__SYSFS_READER_HPP_

#include <unistd.h>

#include <string>

namespace poirot {
namespace utils {

/**
 * @class SysfsReader
 * @brief Static utility class for reading sysfs files.
 *
 * Provides methods for reading different types of values from
 * sysfs virtual filesystem entries commonly used for system monitoring.
 */
class SysfsReader {
public:
  SysfsReader() = delete;

  /**
   * @brief Read a long integer value from a sysfs file.
   * @param path Path to the sysfs file.
   * @return The value read, or -1 on failure.
   */
  static long read_long(const std::string &path);

  /**
   * @brief Read a double value from a sysfs file.
   * @param path Path to the sysfs file.
   * @return The value read, or 0.0 on failure.
   */
  static double read_double(const std::string &path);

  /**
   * @brief Read a string value from a sysfs file.
   * @param path Path to the sysfs file.
   * @return The value read, or empty string on failure.
   */
  static std::string read_string(const std::string &path);

  /**
   * @brief Get the path to a thread's status file.
   * @param filename The filename within the task directory.
   * @return Path to the thread's status file.
   */
  static std::string get_thread_status_path(const std::string &filename);
};

} // namespace utils
} // namespace poirot

#endif // POIROT__UTILS__SYSFS_READER_HPP_
