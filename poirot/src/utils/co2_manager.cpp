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

  // Map to store the most recent data for each country
  std::map<std::string, std::pair<int, double>> country_data;

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

    // Expected format: Entity,Code,Year,Lifecycle carbon intensity of
    // electricity - gCO2/kWh
    if (columns.size() < 4) {
      continue;
    }

    std::string entity = columns[0];
    std::string code = columns[1];
    std::string year_str = columns[2];
    std::string intensity_str = columns[3];

    // Skip if no country code or invalid data
    if (code.empty() || intensity_str.empty()) {
      continue;
    }

    try {
      int year = std::stoi(year_str);
      double intensity = std::stod(intensity_str);
      // Convert from gCO2/kWh to kgCO2/kWh
      double intensity_kg = intensity / 1000.0;

      // Keep the most recent year for each country
      auto it = country_data.find(code);
      if (it == country_data.end() || year > it->second.first) {
        country_data[code] = std::make_pair(year, intensity_kg);
      }
    } catch (...) {
      // Skip invalid rows
      continue;
    }
  }

  // Store the most recent values
  for (const auto &[code, data] : country_data) {
    this->co2_factors_by_country_[code] = data.second;
  }
}

bool Co2Manager::download_factors() {
  CURL *curl = curl_easy_init();
  if (!curl) {
    return false;
  }

  std::string response_data;

  // Try to download from Our World in Data carbon intensity electricity data
  const char *url =
      "https://ourworldindata.org/grapher/carbon-intensity-electricity.csv";

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

  // Try direct match (handles both 2-letter and 3-letter codes)
  auto it = this->co2_factors_by_country_.find(country_code);
  if (it != this->co2_factors_by_country_.end()) {
    return it->second;
  }

  // ISO 2-letter to 3-letter mapping
  static const std::map<std::string, std::string> iso2_to_iso3 = {
      {"AF", "AFG"}, {"AL", "ALB"}, {"DZ", "DZA"}, {"AS", "ASM"}, {"AD", "AND"},
      {"AO", "AGO"}, {"AI", "AIA"}, {"AQ", "ATA"}, {"AG", "ATG"}, {"AR", "ARG"},
      {"AM", "ARM"}, {"AW", "ABW"}, {"AU", "AUS"}, {"AT", "AUT"}, {"AZ", "AZE"},
      {"BS", "BHS"}, {"BH", "BHR"}, {"BD", "BGD"}, {"BB", "BRB"}, {"BY", "BLR"},
      {"BE", "BEL"}, {"BZ", "BLZ"}, {"BJ", "BEN"}, {"BM", "BMU"}, {"BT", "BTN"},
      {"BO", "BOL"}, {"BA", "BIH"}, {"BW", "BWA"}, {"BV", "BVT"}, {"BR", "BRA"},
      {"IO", "IOT"}, {"BN", "BRN"}, {"BG", "BGR"}, {"BF", "BFA"}, {"BI", "BDI"},
      {"KH", "KHM"}, {"CM", "CMR"}, {"CA", "CAN"}, {"CV", "CPV"}, {"KY", "CYM"},
      {"CF", "CAF"}, {"TD", "TCD"}, {"CL", "CHL"}, {"CN", "CHN"}, {"CX", "CXR"},
      {"CC", "CCK"}, {"CO", "COL"}, {"KM", "COM"}, {"CG", "COG"}, {"CD", "COD"},
      {"CK", "COK"}, {"CR", "CRI"}, {"CI", "CIV"}, {"HR", "HRV"}, {"CU", "CUB"},
      {"CY", "CYP"}, {"CZ", "CZE"}, {"DK", "DNK"}, {"DJ", "DJI"}, {"DM", "DMA"},
      {"DO", "DOM"}, {"EC", "ECU"}, {"EG", "EGY"}, {"SV", "SLV"}, {"GQ", "GNQ"},
      {"ER", "ERI"}, {"EE", "EST"}, {"ET", "ETH"}, {"FK", "FLK"}, {"FO", "FRO"},
      {"FJ", "FJI"}, {"FI", "FIN"}, {"FR", "FRA"}, {"GF", "GUF"}, {"PF", "PYF"},
      {"TF", "ATF"}, {"GA", "GAB"}, {"GM", "GMB"}, {"GE", "GEO"}, {"DE", "DEU"},
      {"GH", "GHA"}, {"GI", "GIB"}, {"GR", "GRC"}, {"GL", "GRL"}, {"GD", "GRD"},
      {"GP", "GLP"}, {"GU", "GUM"}, {"GT", "GTM"}, {"GG", "GGY"}, {"GN", "GIN"},
      {"GW", "GNB"}, {"GY", "GUY"}, {"HT", "HTI"}, {"HM", "HMD"}, {"VA", "VAT"},
      {"HN", "HND"}, {"HK", "HKG"}, {"HU", "HUN"}, {"IS", "ISL"}, {"IN", "IND"},
      {"ID", "IDN"}, {"IR", "IRN"}, {"IQ", "IRQ"}, {"IE", "IRL"}, {"IM", "IMN"},
      {"IL", "ISR"}, {"IT", "ITA"}, {"JM", "JAM"}, {"JP", "JPN"}, {"JE", "JEY"},
      {"JO", "JOR"}, {"KZ", "KAZ"}, {"KE", "KEN"}, {"KI", "KIR"}, {"KP", "PRK"},
      {"KR", "KOR"}, {"KW", "KWT"}, {"KG", "KGZ"}, {"LA", "LAO"}, {"LV", "LVA"},
      {"LB", "LBN"}, {"LS", "LSO"}, {"LR", "LBR"}, {"LY", "LBY"}, {"LI", "LIE"},
      {"LT", "LTU"}, {"LU", "LUX"}, {"MO", "MAC"}, {"MK", "MKD"}, {"MG", "MDG"},
      {"MW", "MWI"}, {"MY", "MYS"}, {"MV", "MDV"}, {"ML", "MLI"}, {"MT", "MLT"},
      {"MH", "MHL"}, {"MQ", "MTQ"}, {"MR", "MRT"}, {"MU", "MUS"}, {"YT", "MYT"},
      {"MX", "MEX"}, {"FM", "FSM"}, {"MD", "MDA"}, {"MC", "MCO"}, {"MN", "MNG"},
      {"ME", "MNE"}, {"MS", "MSR"}, {"MA", "MAR"}, {"MZ", "MOZ"}, {"MM", "MMR"},
      {"NA", "NAM"}, {"NR", "NRU"}, {"NP", "NPL"}, {"NL", "NLD"}, {"NC", "NCL"},
      {"NZ", "NZL"}, {"NI", "NIC"}, {"NE", "NER"}, {"NG", "NGA"}, {"NU", "NIU"},
      {"NF", "NFK"}, {"MP", "MNP"}, {"NO", "NOR"}, {"OM", "OMN"}, {"PK", "PAK"},
      {"PW", "PLW"}, {"PS", "PSE"}, {"PA", "PAN"}, {"PG", "PNG"}, {"PY", "PRY"},
      {"PE", "PER"}, {"PH", "PHL"}, {"PN", "PCN"}, {"PL", "POL"}, {"PT", "PRT"},
      {"PR", "PRI"}, {"QA", "QAT"}, {"RE", "REU"}, {"RO", "ROU"}, {"RU", "RUS"},
      {"RW", "RWA"}, {"BL", "BLM"}, {"SH", "SHN"}, {"KN", "KNA"}, {"LC", "LCA"},
      {"MF", "MAF"}, {"PM", "SPM"}, {"VC", "VCT"}, {"WS", "WSM"}, {"SM", "SMR"},
      {"ST", "STP"}, {"SA", "SAU"}, {"SN", "SEN"}, {"RS", "SRB"}, {"SC", "SYC"},
      {"SL", "SLE"}, {"SG", "SGP"}, {"SX", "SXM"}, {"SK", "SVK"}, {"SI", "SVN"},
      {"SB", "SLB"}, {"SO", "SOM"}, {"ZA", "ZAF"}, {"GS", "SGS"}, {"SS", "SSD"},
      {"ES", "ESP"}, {"LK", "LKA"}, {"SD", "SDN"}, {"SR", "SUR"}, {"SJ", "SJM"},
      {"SZ", "SWZ"}, {"SE", "SWE"}, {"CH", "CHE"}, {"SY", "SYR"}, {"TW", "TWN"},
      {"TJ", "TJK"}, {"TZ", "TZA"}, {"TH", "THA"}, {"TL", "TLS"}, {"TG", "TGO"},
      {"TK", "TKL"}, {"TO", "TON"}, {"TT", "TTO"}, {"TN", "TUN"}, {"TR", "TUR"},
      {"TM", "TKM"}, {"TC", "TCA"}, {"TV", "TUV"}, {"UG", "UGA"}, {"UA", "UKR"},
      {"AE", "ARE"}, {"GB", "GBR"}, {"US", "USA"}, {"UM", "UMI"}, {"UY", "URY"},
      {"UZ", "UZB"}, {"VU", "VUT"}, {"VE", "VEN"}, {"VN", "VNM"}, {"VG", "VGB"},
      {"VI", "VIR"}, {"WF", "WLF"}, {"EH", "ESH"}, {"YE", "YEM"}, {"ZM", "ZMB"},
      {"ZW", "ZWE"}};

  // Try 2-letter to 3-letter conversion
  if (country_code.length() == 2) {
    auto iso_it = iso2_to_iso3.find(country_code);
    if (iso_it != iso2_to_iso3.end()) {
      auto factor_it = this->co2_factors_by_country_.find(iso_it->second);
      if (factor_it != this->co2_factors_by_country_.end()) {
        return factor_it->second;
      }
    }
  }

  // Try 3-letter to 2-letter conversion
  if (country_code.length() == 3) {
    for (const auto &[iso2, iso3] : iso2_to_iso3) {
      if (iso3 == country_code) {
        auto factor_it = this->co2_factors_by_country_.find(iso3);
        if (factor_it != this->co2_factors_by_country_.end()) {
          return factor_it->second;
        }
      }
    }
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

} // namespace utils
} // namespace poirot
