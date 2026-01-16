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

#ifndef POIROT__UTILS__CO2_MANAGER_HPP_
#define POIROT__UTILS__CO2_MANAGER_HPP_

#include <map>
#include <mutex>
#include <string>

namespace poirot {
namespace utils {

/// @brief Default CO2 factor in kg CO2 per kWh (global average)
constexpr double DEFAULT_CO2_FACTOR_KG_PER_KWH = 0.436;
/// @brief Timeout for CURL requests in seconds
constexpr long CURL_TIMEOUT_SECONDS = 3;

/**
 * @class Co2Manager
 * @brief Class for managing CO2 emission factors.
 *
 * Provides methods for downloading CO2 factors from online sources
 * and looking up factors by country.
 */
class Co2Manager {
public:
  /**
   * @brief Constructor.
   */
  Co2Manager() = default;

  /**
   * @brief Download CO2 factors from online source.
   * @return True if download succeeded, false otherwise.
   */
  bool download_factors();

  /**
   * @brief Get CO2 factor for a specific country.
   * @param country_code ISO 2-letter country code.
   * @return CO2 factor in kg CO2 per kWh.
   */
  double get_factor_for_country(const std::string &country_code) const;

  /**
   * @brief Get country code from timezone.
   * @param timezone Timezone string (e.g., "Europe/Madrid").
   * @return ISO 2-letter country code.
   */
  std::string get_country_from_timezone(const std::string &timezone);

  /**
   * @brief Get the current system timezone.
   * @return Timezone string.
   */
  std::string get_system_timezone();

  /**
   * @brief Check if CO2 factors have been loaded.
   * @return True if factors are loaded.
   */
  bool factors_loaded() const { return this->co2_factors_loaded_; }

  /**
   * @brief Get the number of loaded country factors.
   * @return Number of countries with CO2 factors.
   */
  size_t get_factor_count() const {
    std::lock_guard<std::mutex> lock(this->co2_factors_mutex_);
    return this->co2_factors_by_country_.size();
  }

private:
  /// @brief Map of country code to CO2 factor
  std::map<std::string, double> co2_factors_by_country_;
  /// @brief Mutex for thread-safe access to CO2 factors
  mutable std::mutex co2_factors_mutex_;
  /// @brief Flag indicating if CO2 factors have been loaded
  bool co2_factors_loaded_ = false;
  /// @brief Cache for timezone to country mapping
  std::map<std::string, std::string> timezone_to_country_;
  /// @brief Flag indicating if timezone mapping has been loaded
  bool timezone_map_loaded_ = false;

  /**
   * @brief CURL write callback function.
   * @param contents Data received.
   * @param size Size of each element.
   * @param nmemb Number of elements.
   * @param userp User pointer to output string.
   * @return Number of bytes written.
   */
  static size_t curl_write_callback(void *contents, size_t size, size_t nmemb,
                                    void *userp);

  /**
   * @brief Parse CSV data to extract CO2 factors.
   * @param csv_data Raw CSV data string.
   */
  void parse_csv_data(const std::string &csv_data);

  /**
   * @brief Load timezone to country mapping from system zone.tab file.
   */
  void load_timezone_mapping();
};

} // namespace utils
} // namespace poirot

#endif // POIROT__UTILS__CO2_MANAGER_HPP_
