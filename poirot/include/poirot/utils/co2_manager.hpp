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
/// @brief Ember API base URL
constexpr const char *EMBER_API_BASE_URL = "https://api.ember-energy.org/v1";

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
  Co2Manager();

  /**
   * @brief Get CO2 factor for a specific country.
   * @param country_code ISO 2-letter country code.
   * @return CO2 factor in kg CO2 per kWh.
   */
  double get_co2_factor(const std::string &country_code);

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

private:
  /// @brief Cache for timezone to country mapping
  std::map<std::string, std::string> timezone_to_country_;
  /// @brief Flag indicating if timezone mapping has been loaded
  bool timezone_map_loaded_ = false;
  /// @brief ISO2 to ISO3 mapping
  std::map<std::string, std::string> iso2_to_iso3_;
  /// @brief Flag indicating if ISO mapping has been loaded
  bool iso_map_loaded_ = false;

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
   * @brief Load timezone to country mapping from system zone.tab file.
   */
  void load_timezone_mapping();

  /**
   * @brief Load ISO mapping from installed file.
   */
  void load_iso_mapping();

  /**
   * @brief Parse CSV data to extract ISO mappings.
   * @param csv_data Raw CSV data string.
   */
  void parse_iso_csv(const std::string &csv_data);
};

} // namespace utils
} // namespace poirot

#endif // POIROT__UTILS__CO2_MANAGER_HPP_
