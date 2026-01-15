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

#ifndef POIROT__UTILS__STRING_UTILS_HPP_
#define POIROT__UTILS__STRING_UTILS_HPP_

#include <string>

namespace poirot {
namespace utils {

/**
 * @class StringUtils
 * @brief Static utility class for string operations.
 *
 * Provides utility methods for string manipulation and generation.
 */
class StringUtils {
public:
  // Delete constructor to prevent instantiation
  StringUtils() = delete;

  /**
   * @brief Generates a unique UUID as a string.
   *
   * This function uses random numbers to generate a 16-character hexadecimal
   * UUID.
   *
   * @return A string containing a 16-character hexadecimal UUID.
   */
  static std::string generate_uuid();

  /**
   * @brief Trim whitespace from both ends of a string.
   * @param str The string to trim.
   * @return The trimmed string.
   */
  static std::string trim(const std::string &str);

  /**
   * @brief Convert string to lowercase.
   * @param str The string to convert.
   * @return The lowercase string.
   */
  static std::string to_lower(const std::string &str);

  /**
   * @brief Convert string to uppercase.
   * @param str The string to convert.
   * @return The uppercase string.
   */
  static std::string to_upper(const std::string &str);
};

} // namespace utils
} // namespace poirot

#endif // POIROT__UTILS__STRING_UTILS_HPP_
