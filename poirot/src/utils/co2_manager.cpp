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

#include <curl/curl.h>
#include <unistd.h>

#include <algorithm>
#include <cstring>
#include <ctime>
#include <fstream>
#include <sstream>
#include <vector>

#include "poirot/utils/co2_manager.hpp"

namespace poirot {
namespace utils {

size_t Co2Manager::curl_write_callback(void *contents, size_t size,
                                       size_t nmemb, void *userp) {
  size_t total_size = size * nmemb;
  std::string *str = static_cast<std::string *>(userp);
  str->append(static_cast<char *>(contents), total_size);
  return total_size;
}

void Co2Manager::parse_csv_data(const std::string &csv_data) {
  std::istringstream stream(csv_data);
  std::string line;
  bool first_line = true;

  // Find column indices
  int country_col = -1;
  int co2_col = -1;

  while (std::getline(stream, line)) {
    if (line.empty()) {
      continue;
    }

    std::istringstream line_stream(line);
    std::string cell;
    std::vector<std::string> columns;

    while (std::getline(line_stream, cell, ',')) {
      // Remove quotes if present
      if (cell.size() >= 2 && cell.front() == '"' && cell.back() == '"') {
        cell = cell.substr(1, cell.size() - 2);
      }
      columns.push_back(cell);
    }

    if (first_line) {
      first_line = false;
      // Find column indices
      for (size_t i = 0; i < columns.size(); ++i) {
        std::string header = columns[i];
        std::transform(header.begin(), header.end(), header.begin(), ::tolower);
        if (header.find("country") != std::string::npos ||
            header.find("iso") != std::string::npos) {
          country_col = static_cast<int>(i);
        } else if (header.find("co2") != std::string::npos ||
                   header.find("carbon") != std::string::npos ||
                   header.find("intensity") != std::string::npos) {
          co2_col = static_cast<int>(i);
        }
      }
      continue;
    }

    // Parse data rows
    if (country_col >= 0 && co2_col >= 0 &&
        country_col < static_cast<int>(columns.size()) &&
        co2_col < static_cast<int>(columns.size())) {
      std::string country = columns[country_col];
      try {
        double co2_factor = std::stod(columns[co2_col]);
        // Convert from g/kWh to kg/kWh if necessary
        if (co2_factor > 10.0) {
          co2_factor /= 1000.0;
        }
        this->co2_factors_by_country_[country] = co2_factor;
      } catch (...) {
        // Skip invalid rows
      }
    }
  }
}

bool Co2Manager::download_factors() {
  CURL *curl = curl_easy_init();
  if (!curl) {
    return false;
  }

  std::string response_data;

  // Try to download from Ember's public data
  const char *url =
      "https://ember-climate.org/app/uploads/2022/07/yearly_full_release_long_"
      "format-4.csv";

  curl_easy_setopt(curl, CURLOPT_URL, url);
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, curl_write_callback);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response_data);
  curl_easy_setopt(curl, CURLOPT_TIMEOUT, CURL_TIMEOUT_SECONDS);
  curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
  curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 1L);

  CURLcode res = curl_easy_perform(curl);
  curl_easy_cleanup(curl);

  if (res != CURLE_OK) {
    return false;
  }

  // Parse the CSV data
  {
    std::lock_guard<std::mutex> lock(this->co2_factors_mutex_);
    this->parse_csv_data(response_data);
    this->co2_factors_loaded_ = !this->co2_factors_by_country_.empty();
  }

  return this->co2_factors_loaded_;
}

double
Co2Manager::get_factor_for_country(const std::string &country_code) const {
  std::lock_guard<std::mutex> lock(this->co2_factors_mutex_);

  auto it = this->co2_factors_by_country_.find(country_code);
  if (it != this->co2_factors_by_country_.end()) {
    return it->second;
  }

  // Try lowercase
  std::string lower_code = country_code;
  std::transform(lower_code.begin(), lower_code.end(), lower_code.begin(),
                 ::tolower);
  it = this->co2_factors_by_country_.find(lower_code);
  if (it != this->co2_factors_by_country_.end()) {
    return it->second;
  }

  // Try uppercase
  std::string upper_code = country_code;
  std::transform(upper_code.begin(), upper_code.end(), upper_code.begin(),
                 ::toupper);
  it = this->co2_factors_by_country_.find(upper_code);
  if (it != this->co2_factors_by_country_.end()) {
    return it->second;
  }

  return DEFAULT_CO2_FACTOR_KG_PER_KWH;
}

std::string Co2Manager::get_system_timezone() {
  // Try to read from /etc/timezone
  std::ifstream tz_file("/etc/timezone");
  if (tz_file.is_open()) {
    std::string timezone;
    std::getline(tz_file, timezone);
    if (!timezone.empty()) {
      return timezone;
    }
  }

  // Try to read from /etc/localtime symlink
  char buf[256];
  ssize_t len = readlink("/etc/localtime", buf, sizeof(buf) - 1);
  if (len > 0) {
    buf[len] = '\0';
    std::string path(buf);
    size_t pos = path.find("zoneinfo/");
    if (pos != std::string::npos) {
      return path.substr(pos + 9);
    }
  }

  // Fallback to TZ environment variable
  const char *tz_env = std::getenv("TZ");
  if (tz_env && tz_env[0] != '\0') {
    return std::string(tz_env);
  }

  return "UTC";
}

std::string Co2Manager::get_country_from_timezone(const std::string &timezone) {
  // Mapping of timezone regions to country codes
  static const std::map<std::string, std::string> timezone_to_country = {
      // Europe
      {"Europe/Madrid", "ES"},
      {"Europe/Paris", "FR"},
      {"Europe/London", "GB"},
      {"Europe/Berlin", "DE"},
      {"Europe/Rome", "IT"},
      {"Europe/Amsterdam", "NL"},
      {"Europe/Brussels", "BE"},
      {"Europe/Zurich", "CH"},
      {"Europe/Vienna", "AT"},
      {"Europe/Warsaw", "PL"},
      {"Europe/Prague", "CZ"},
      {"Europe/Budapest", "HU"},
      {"Europe/Stockholm", "SE"},
      {"Europe/Oslo", "NO"},
      {"Europe/Copenhagen", "DK"},
      {"Europe/Helsinki", "FI"},
      {"Europe/Dublin", "IE"},
      {"Europe/Lisbon", "PT"},
      {"Europe/Athens", "GR"},
      {"Europe/Moscow", "RU"},
      // Americas
      {"America/New_York", "US"},
      {"America/Los_Angeles", "US"},
      {"America/Chicago", "US"},
      {"America/Denver", "US"},
      {"America/Phoenix", "US"},
      {"America/Toronto", "CA"},
      {"America/Vancouver", "CA"},
      {"America/Montreal", "CA"},
      {"America/Mexico_City", "MX"},
      {"America/Sao_Paulo", "BR"},
      {"America/Buenos_Aires", "AR"},
      {"America/Santiago", "CL"},
      {"America/Lima", "PE"},
      {"America/Bogota", "CO"},
      // Asia
      {"Asia/Tokyo", "JP"},
      {"Asia/Shanghai", "CN"},
      {"Asia/Hong_Kong", "HK"},
      {"Asia/Singapore", "SG"},
      {"Asia/Seoul", "KR"},
      {"Asia/Taipei", "TW"},
      {"Asia/Bangkok", "TH"},
      {"Asia/Jakarta", "ID"},
      {"Asia/Manila", "PH"},
      {"Asia/Kolkata", "IN"},
      {"Asia/Mumbai", "IN"},
      {"Asia/Dubai", "AE"},
      {"Asia/Jerusalem", "IL"},
      // Oceania
      {"Australia/Sydney", "AU"},
      {"Australia/Melbourne", "AU"},
      {"Australia/Brisbane", "AU"},
      {"Australia/Perth", "AU"},
      {"Pacific/Auckland", "NZ"},
      // Africa
      {"Africa/Cairo", "EG"},
      {"Africa/Johannesburg", "ZA"},
      {"Africa/Lagos", "NG"},
      {"Africa/Nairobi", "KE"},
  };

  auto it = timezone_to_country.find(timezone);
  if (it != timezone_to_country.end()) {
    return it->second;
  }

  // Try to extract country from timezone path (e.g., "Europe/Madrid" -> search
  // for partial match)
  for (const auto &[tz, country] : timezone_to_country) {
    if (timezone.find(tz) != std::string::npos ||
        tz.find(timezone) != std::string::npos) {
      return country;
    }
  }

  return "UNKNOWN";
}

} // namespace utils
} // namespace poirot
