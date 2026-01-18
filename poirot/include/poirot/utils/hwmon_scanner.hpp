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

#ifndef POIROT__UTILS__HWMON_SCANNER_HPP_
#define POIROT__UTILS__HWMON_SCANNER_HPP_

#include <functional>
#include <string>

namespace poirot {
namespace utils {

/**
 * @class HwmonScanner
 * @brief Class for scanning and caching hwmon device paths.
 *
 * Provides methods for iterating over hwmon devices and caching
 * paths for energy and power readings.
 */
class HwmonScanner {
public:
  /**
   * @brief Constructor.
   */
  HwmonScanner();

  /**
   * @brief Iterate over hwmon devices.
   * @param callback Function called for each device (base_path, name).
   *                 Return true to stop iteration.
   */
  void iterate_devices(
      const std::function<bool(const std::string &, const std::string &)>
          &callback);

  /**
   * @brief Search for and cache hwmon paths for energy and power readings.
   */
  void search_paths();

  /**
   * @brief Read power from cached hwmon path.
   * @return Power in watts or 0.0 if not available.
   */
  double read_power_w() const;

  /**
   * @brief Get cached energy path.
   * @return Cached energy path or empty string.
   */
  const std::string &get_energy_path() const {
    return this->cached_energy_path_;
  }

  /**
   * @brief Get cached power path.
   * @return Cached power path or empty string.
   */
  const std::string &get_power_path() const { return this->cached_power_path_; }

  /**
   * @brief Check if paths have been searched.
   * @return True if search has been performed.
   */
  bool is_searched() const { return this->paths_searched_; }

private:
  /// @brief Cached path for hwmon energy readings
  std::string cached_energy_path_;
  /// @brief Cached path for hwmon power readings
  std::string cached_power_path_;
  /// @brief Flag indicating if paths have been searched
  bool paths_searched_ = false;
};

} // namespace utils
} // namespace poirot

#endif // POIROT__UTILS__HWMON_SCANNER_HPP_
