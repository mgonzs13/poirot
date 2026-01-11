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

#include <dirent.h>
#include <fstream>

#include "poirot/utils/hwmon_scanner.hpp"
#include "poirot/utils/sysfs_reader.hpp"

namespace poirot {
namespace utils {

void HwmonScanner::iterate_devices(
    const std::function<bool(const std::string &, const std::string &)>
        &callback) {
  DIR *dir = opendir("/sys/class/hwmon");
  if (!dir) {
    return;
  }

  struct dirent *entry;
  while ((entry = readdir(dir)) != nullptr) {
    if (entry->d_name[0] == '.') {
      continue;
    }

    std::string base = std::string("/sys/class/hwmon/") + entry->d_name;
    std::string name = SysfsReader::read_string(base + "/name");

    if (callback(base, name)) {
      break;
    }
  }

  closedir(dir);
}

void HwmonScanner::search_paths() {
  if (this->paths_searched_) {
    return;
  }

  this->paths_searched_ = true;

  this->iterate_devices([this](const std::string &base,
                               const std::string &name) {
    bool is_energy_driver = (name == "zenpower" || name == "amd_energy" ||
                             name.find("energy") != std::string::npos);
    bool is_power_driver =
        (name == "zenpower" || name == "amd_energy" || name == "coretemp" ||
         name == "k10temp" || name.find("power") != std::string::npos);

    if (is_energy_driver && this->cached_energy_path_.empty()) {
      for (int i = 1; i <= 10; ++i) {
        std::string energy_path =
            base + "/energy" + std::to_string(i) + "_input";
        if (SysfsReader::read_long(energy_path) >= 0 ||
            std::ifstream(energy_path).is_open()) {
          this->cached_energy_path_ = energy_path;
          break;
        }
      }
    }

    if (is_power_driver && this->cached_power_path_.empty()) {
      for (int i = 1; i <= 10; ++i) {
        std::string power_path = base + "/power" + std::to_string(i) + "_input";
        if (std::ifstream(power_path).is_open()) {
          this->cached_power_path_ = power_path;
          break;
        }
      }
    }

    return !this->cached_energy_path_.empty() &&
           !this->cached_power_path_.empty();
  });
}

double HwmonScanner::read_power_w() const {
  if (this->cached_power_path_.empty()) {
    return 0.0;
  }

  long power_uw = SysfsReader::read_long(this->cached_power_path_);
  if (power_uw > 0) {
    return static_cast<double>(power_uw) / 1e6;
  }

  return 0.0;
}

} // namespace utils
} // namespace poirot
