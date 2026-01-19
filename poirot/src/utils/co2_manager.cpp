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
#include <rapidjson/document.h>
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

Co2Manager::Co2Manager(std::string iso_country_codes_file_path) {
  this->load_iso_mapping(iso_country_codes_file_path);
}

size_t Co2Manager::curl_write_callback(void *contents, size_t size,
                                       size_t nmemb, void *userp) {
  size_t total_size = size * nmemb;
  std::string *str = static_cast<std::string *>(userp);
  str->append(static_cast<char *>(contents), total_size);
  return total_size;
}

Co2Info Co2Manager::get_co2_info() {
  std::string timezone = this->get_system_timezone();
  std::string country_code = this->get_country_from_timezone(timezone);
  return this->get_co2_factor(country_code);
}

Co2Info Co2Manager::get_co2_factor(const std::string &country_code) {

  Co2Info co2_info;
  co2_info.country_code = country_code;

  // Get API key from environment
  const char *api_key = std::getenv("EMBER_KEY");
  if (!api_key) {
    return co2_info;
  }

  // Convert to ISO3
  std::string iso3 = country_code;
  if (country_code.length() == 2 && this->iso_map_loaded_) {
    auto it = this->iso2_to_iso3_.find(country_code);
    if (it != this->iso2_to_iso3_.end()) {
      iso3 = it->second;
    } else {
      return co2_info;
    }
  }

  CURL *curl = curl_easy_init();
  if (!curl) {
    return co2_info;
  }

  std::string response_data;

  // Construct API URL
  std::string url = std::string(EMBER_API_BASE_URL) +
                    "/carbon-intensity/monthly?api_key=" + api_key +
                    "&entity_code=" + iso3;

  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, curl_write_callback);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response_data);
  curl_easy_setopt(curl, CURLOPT_TIMEOUT, CURL_TIMEOUT_SECONDS);
  curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
  curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 1L);

  CURLcode res = curl_easy_perform(curl);
  curl_easy_cleanup(curl);

  if (res != CURLE_OK) {
    return co2_info;
  }

  // Parse the JSON data
  rapidjson::Document j;
  if (j.Parse(response_data.c_str()).HasParseError()) {
    return co2_info;
  }

  if (j.HasMember("data") && j["data"].IsArray() && !j["data"].Empty()) {
    // Get the last (most recent) data point
    const rapidjson::Value &last_entry = j["data"][j["data"].Size() - 1];
    if (last_entry.HasMember("emissions_intensity_gco2_per_kwh") &&
        last_entry["emissions_intensity_gco2_per_kwh"].IsNumber()) {
      double value = last_entry["emissions_intensity_gco2_per_kwh"].GetDouble();
      co2_info.co2_factor_loaded = true;
      co2_info.co2_factor_kg_per_kwh = value / 1000.0;
      return co2_info;
    }
  }

  return co2_info;
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
  // Load timezone mapping from zone.tab if not already loaded
  if (!this->timezone_map_loaded_) {
    this->load_timezone_mapping();
  }

  // Try exact match
  auto it = this->timezone_to_country_.find(timezone);
  if (it != this->timezone_to_country_.end()) {
    return it->second;
  }

  // Try partial matches
  for (const auto &[tz, country] : this->timezone_to_country_) {
    if (timezone.find(tz) != std::string::npos ||
        tz.find(timezone) != std::string::npos) {
      return country;
    }
  }

  return "UNKNOWN";
}

void Co2Manager::load_timezone_mapping() {
  // Possible locations for zone.tab file
  const char *zone_tab_paths[] = {
      "/usr/share/zoneinfo/zone.tab",
      "/usr/share/zoneinfo/zone1970.tab",
      "/usr/share/lib/zoneinfo/tab/zone_sun.tab",
  };

  for (const char *zone_tab_path : zone_tab_paths) {
    std::ifstream file(zone_tab_path);
    if (!file.is_open()) {
      continue;
    }

    std::string line;
    while (std::getline(file, line)) {
      // Skip empty lines and comments
      if (line.empty() || line[0] == '#') {
        continue;
      }

      // Parse line: <country_code> <coordinates> <timezone> [<comments>]
      std::istringstream iss(line);
      std::string country_code, coordinates, timezone;

      if (iss >> country_code >> coordinates >> timezone) {
        this->timezone_to_country_[timezone] = country_code;
      }
    }

    // Successfully loaded from this file
    this->timezone_map_loaded_ = true;
    return;
  }

  // If we couldn't load from any file, mark as loaded anyway to avoid repeated
  // attempts
  this->timezone_map_loaded_ = true;
}

void Co2Manager::load_iso_mapping(std::string iso_country_codes_file_path) {
  try {
    std::ifstream file(iso_country_codes_file_path);
    if (!file.is_open()) {
      this->iso_map_loaded_ = false;
      return;
    }

    std::string csv_data((std::istreambuf_iterator<char>(file)),
                         std::istreambuf_iterator<char>());

    this->parse_iso_csv(csv_data);
    this->iso_map_loaded_ = true;
  } catch (...) {
    this->iso_map_loaded_ = false;
  }
}

void Co2Manager::parse_iso_csv(const std::string &csv_data) {
  std::istringstream stream(csv_data);
  std::string line;
  bool first_line = true;

  while (std::getline(stream, line)) {
    if (line.empty()) {
      continue;
    }

    if (first_line) {
      first_line = false;
      continue; // Skip header
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

    if (columns.size() < 3) {
      continue;
    }

    std::string alpha2 = columns[1];
    std::string alpha3 = columns[2];

    if (!alpha2.empty() && !alpha3.empty()) {
      this->iso2_to_iso3_[alpha2] = alpha3;
    }
  }
}

} // namespace utils
} // namespace poirot
