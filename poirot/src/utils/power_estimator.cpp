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

#include <algorithm>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <vector>

#include "poirot/utils/power_estimator.hpp"
#include "poirot/utils/sysfs_reader.hpp"

namespace poirot {
namespace utils {

PowerEstimator::PowerEstimator(HwmonScanner &hwmon_scanner)
    : hwmon_scanner_(hwmon_scanner) {}

std::pair<double, TdpType> PowerEstimator::read_tdp_watts() {

  double tdp_watts = 0.0;
  TdpType tdp_type;

  // 1. Try RAPL power limit
  double rapl_power = this->read_rapl_power_limit_w();
  if (rapl_power > 0) {
    tdp_watts = rapl_power;
    tdp_type = TdpType::RAPL_TDP_TYPE;
  }

  // 2. Try hwmon power limit (TDP from hwmon drivers)
  if (tdp_watts == 0.0) {
    double hwmon_tdp = this->read_hwmon_tdp_watts();
    if (hwmon_tdp > 0) {
      tdp_watts = hwmon_tdp;
      tdp_type = TdpType::HWMON_TDP_TYPE;
    }
  }

  // 3. Try thermal zone power budget
  if (tdp_watts == 0.0) {
    double thermal_tdp = this->read_thermal_tdp_watts();
    if (thermal_tdp > 0) {
      tdp_watts = thermal_tdp;
      tdp_type = TdpType::THERMAL_POWER_TDP_TYPE;
    }
  }

  return {tdp_watts, tdp_type};
}

double PowerEstimator::read_rapl_power_limit_w() {

  static const std::vector<std::string> intel_rapl_paths = {
      "/sys/class/powercap/intel-rapl/intel-rapl:0/"
      "constraint_0_power_limit_uw",
      "/sys/class/powercap/intel-rapl/intel-rapl:0/"
      "constraint_1_power_limit_uw"};

  for (const auto &path : intel_rapl_paths) {
    long power_uw = SysfsReader::read_long(path);
    if (power_uw > 0) {
      return static_cast<double>(power_uw) / 1e6;
    }
  }

  return 0.0;
}

double PowerEstimator::read_hwmon_tdp_watts() {
  return this->hwmon_scanner_.read_power_w();
}

double PowerEstimator::read_thermal_tdp_watts() {
  double tdp_watts = 0.0;

  DIR *thermal_dir = opendir("/sys/class/thermal");
  if (!thermal_dir) {
    return 0.0;
  }

  struct dirent *entry;
  // Look for cooling devices related to the CPU
  while ((entry = readdir(thermal_dir)) != nullptr) {
    if (strncmp(entry->d_name, "cooling_device", 14) == 0) {
      std::string base = std::string("/sys/class/thermal/") + entry->d_name;
      std::string type = SysfsReader::read_string(base + "/type");

      if (type.find("Processor") != std::string::npos ||
          type.find("processor") != std::string::npos) {

        long max_pstate = SysfsReader::read_long(base + "/max_state");
        if (max_pstate > 0) {
          // Try sustainable power from thermal zone
          long power_mw = SysfsReader::read_long(
              "/sys/class/thermal/thermal_zone0/sustainable_power");

          if (power_mw > 0) {
            tdp_watts = static_cast<double>(power_mw) / 1000.0;
          }

          // Try cooling device power
          if (tdp_watts == 0.0) {
            power_mw = SysfsReader::read_long(base + "/power");
            if (power_mw > 0) {
              tdp_watts = static_cast<double>(power_mw) / 1000.0;
            }
          }
        }
      }
    }
  }

  closedir(thermal_dir);
  return tdp_watts;
}

} // namespace utils
} // namespace poirot
