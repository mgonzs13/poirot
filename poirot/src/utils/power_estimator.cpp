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

  // 4. Estimate from CPU frequency and core count (physics-based)
  if (tdp_watts == 0.0) {
    double freq_tdp = this->estimate_frequency_tdp_watts();
    if (freq_tdp > 0) {
      tdp_watts = freq_tdp;
      tdp_type = TdpType::CPU_CORES_FREQUENCY_TYPE;
    }
  }

  // 5. Final fallback: estimate from core count alone
  if (tdp_watts == 0.0) {
    tdp_watts = this->estimate_cores_tdp_watts();
    tdp_type = TdpType::CPU_CORES_TYPE;
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

double PowerEstimator::read_battery_power_w() {
  DIR *dir = opendir("/sys/class/power_supply");
  if (!dir) {
    return -1.0;
  }

  struct dirent *entry;
  while ((entry = readdir(dir)) != nullptr) {
    if (strncmp(entry->d_name, "BAT", 3) == 0) {
      std::string base =
          std::string("/sys/class/power_supply/") + entry->d_name;

      // Try power_now first (direct power measurement)
      std::ifstream power_file(base + "/power_now");
      if (power_file.is_open()) {
        long power_uw = 0;
        power_file >> power_uw;
        if (power_uw > 0) {
          closedir(dir);
          return static_cast<double>(power_uw) / 1e6;
        }
      }

      // Fallback: calculate from voltage and current
      std::ifstream voltage_file(base + "/voltage_now");
      std::ifstream current_file(base + "/current_now");
      if (voltage_file.is_open() && current_file.is_open()) {
        long voltage_uv = 0;
        long current_ua = 0;
        voltage_file >> voltage_uv;
        current_file >> current_ua;

        if (voltage_uv > 0 && current_ua > 0) {
          double power_w = (static_cast<double>(voltage_uv) *
                            static_cast<double>(current_ua)) /
                           1e12;
          closedir(dir);
          return power_w;
        }
      }
    }
  }

  closedir(dir);
  return -1.0;
}

double PowerEstimator::read_min_tdp_watts() {
  static const std::vector<std::string> min_power_paths = {
      "/sys/class/powercap/intel-rapl/intel-rapl:0/constraint_0_min_power_uw",
      "/sys/class/powercap/intel-rapl/intel-rapl:0/constraint_1_min_power_uw",
  };

  for (const auto &path : min_power_paths) {
    long power_uw = SysfsReader::read_long(path);
    if (power_uw > 0) {
      return static_cast<double>(power_uw) / 1e6;
    }
  }

  // Estimate from minimum CPU frequency
  long min_freq_khz = SysfsReader::read_long(
      "/sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_min_freq");
  if (min_freq_khz > 0 && this->cpu_cores_ > 0) {
    double min_freq_ghz = static_cast<double>(min_freq_khz) / 1e6;
    double watts_per_ghz = this->read_watts_per_ghz();
    double min_tdp = this->cpu_cores_ * min_freq_ghz * watts_per_ghz;
    if (min_tdp > 0) {
      return min_tdp;
    }
  }

  return FALLBACK_MIN_TDP_WATTS;
}

double PowerEstimator::read_max_tdp_watts() {
  static const std::vector<std::string> max_power_paths = {
      "/sys/class/powercap/intel-rapl/intel-rapl:0/constraint_0_max_power_uw",
      "/sys/class/powercap/intel-rapl/intel-rapl:0/constraint_1_max_power_uw",
  };

  for (const auto &path : max_power_paths) {
    long power_uw = SysfsReader::read_long(path);
    if (power_uw > 0) {
      return static_cast<double>(power_uw) / 1e6;
    }
  }

  // Try hwmon power cap
  double max_tdp_w = 0.0;
  this->hwmon_scanner_.iterate_devices(
      [&max_tdp_w](const std::string &base, const std::string &name) {
        if (name == "zenpower" || name == "amd_energy" || name == "coretemp" ||
            name == "k10temp") {
          long power_uw = SysfsReader::read_long(base + "/power1_cap_max");
          if (power_uw > 0) {
            max_tdp_w = static_cast<double>(power_uw) / 1e6;
            return true;
          }
        }
        return false;
      });

  if (max_tdp_w > 0.0) {
    return max_tdp_w;
  }

  // Estimate from max CPU frequency
  long max_freq_khz = SysfsReader::read_long(
      "/sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_max_freq");
  if (max_freq_khz > 0 && this->cpu_cores_ > 0) {
    double max_freq_ghz = static_cast<double>(max_freq_khz) / 1e6;
    double watts_per_ghz = this->read_watts_per_ghz();
    double max_tdp = this->cpu_cores_ * max_freq_ghz * watts_per_ghz;
    if (max_tdp > 0) {
      return max_tdp * 1.5;
    }
  }

  return FALLBACK_MAX_TDP_WATTS;
}

double PowerEstimator::read_idle_power_factor() {
  double idle_power_w = 0.0;
  double max_power_w = this->read_max_tdp_watts();

  long idle_uw =
      SysfsReader::read_long("/sys/class/powercap/intel-rapl/intel-rapl:0/"
                             "constraint_1_power_limit_uw");
  if (idle_uw > 0) {
    idle_power_w = static_cast<double>(idle_uw) / 1e6;
  }

  if (idle_power_w == 0.0) {
    long sustainable_mw = SysfsReader::read_long(
        "/sys/class/thermal/thermal_zone0/sustainable_power");
    if (sustainable_mw > 0) {
      idle_power_w = static_cast<double>(sustainable_mw) / 1000.0;
    }
  }

  if (idle_power_w == 0.0 || max_power_w <= 0.0) {
    long min_freq_khz = SysfsReader::read_long(
        "/sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_min_freq");
    long max_freq_khz = SysfsReader::read_long(
        "/sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_max_freq");
    if (min_freq_khz > 0 && max_freq_khz > 0) {
      double freq_ratio =
          static_cast<double>(min_freq_khz) / static_cast<double>(max_freq_khz);
      return freq_ratio * freq_ratio;
    }
  }

  if (idle_power_w > 0.0 && max_power_w > 0.0) {
    double factor = idle_power_w / max_power_w;
    return std::max(0.05, std::min(0.5, factor));
  }

  return FALLBACK_IDLE_POWER_FACTOR;
}

double PowerEstimator::read_watts_per_ghz() {
  double current_power_w = this->hwmon_scanner_.read_power_w();
  if (current_power_w == 0.0) {
    current_power_w = this->read_rapl_power_limit_w();
  }

  long freq_khz = SysfsReader::read_long(
      "/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq");
  double current_freq_ghz = static_cast<double>(freq_khz) / 1e6;

  if (current_power_w > 0 && current_freq_ghz > 0 && this->cpu_cores_ > 0) {
    return current_power_w / (this->cpu_cores_ * current_freq_ghz);
  }

  if (this->cpu_tdp_watts_ > 0) {
    long max_freq_khz = SysfsReader::read_long(
        "/sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_max_freq");
    if (max_freq_khz > 0 && this->cpu_cores_ > 0) {
      double max_freq_ghz = static_cast<double>(max_freq_khz) / 1e6;
      return this->cpu_tdp_watts_ / (this->cpu_cores_ * max_freq_ghz);
    }
  }

  return FALLBACK_WATTS_PER_GHZ;
}

double PowerEstimator::read_min_watts_per_ghz() {
  double min_tdp = this->read_min_tdp_watts();
  long max_freq_khz = SysfsReader::read_long(
      "/sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_max_freq");

  if (min_tdp > 0 && max_freq_khz > 0 && this->cpu_cores_ > 0) {
    double max_freq_ghz = static_cast<double>(max_freq_khz) / 1e6;
    double min_watts_per_ghz = min_tdp / (this->cpu_cores_ * max_freq_ghz);
    if (min_watts_per_ghz > 0) {
      return min_watts_per_ghz;
    }
  }

  double watts_per_ghz = this->read_watts_per_ghz();
  double idle_factor = this->read_idle_power_factor();
  if (watts_per_ghz > 0 && idle_factor > 0) {
    return watts_per_ghz * idle_factor;
  }

  return FALLBACK_MIN_WATTS_PER_GHZ;
}

double PowerEstimator::read_max_watts_per_ghz() {
  double max_tdp = this->read_max_tdp_watts();
  long base_freq_khz = SysfsReader::read_long(
      "/sys/devices/system/cpu/cpu0/cpufreq/base_frequency");

  if (base_freq_khz <= 0) {
    base_freq_khz = SysfsReader::read_long(
        "/sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq");
  }

  if (max_tdp > 0 && base_freq_khz > 0 && this->cpu_cores_ > 0) {
    double base_freq_ghz = static_cast<double>(base_freq_khz) / 1e6;
    double max_watts_per_ghz = max_tdp / (this->cpu_cores_ * base_freq_ghz);
    if (max_watts_per_ghz > 0) {
      return max_watts_per_ghz;
    }
  }

  double watts_per_ghz = this->read_watts_per_ghz();
  if (watts_per_ghz > 0) {
    return watts_per_ghz * 2.0;
  }

  return FALLBACK_MAX_WATTS_PER_GHZ;
}

double PowerEstimator::estimate_power_per_core_per_ghz() {
  double current_power_w = this->hwmon_scanner_.read_power_w();

  if (current_power_w == 0.0) {
    current_power_w = read_rapl_power_limit_w();
  }

  long freq_khz = SysfsReader::read_long(
      "/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq");
  double current_freq_ghz = static_cast<double>(freq_khz) / 1e6;

  if (current_power_w > 0 && current_freq_ghz > 0 && this->cpu_cores_ > 0) {
    double calculated = current_power_w / (this->cpu_cores_ * current_freq_ghz);
    double min_watts_per_ghz = this->read_min_watts_per_ghz();
    double max_watts_per_ghz = this->read_max_watts_per_ghz();
    if (calculated >= min_watts_per_ghz && calculated <= max_watts_per_ghz) {
      return calculated;
    }
  }

  long max_speed_mhz =
      SysfsReader::read_long("/sys/class/dmi/id/processor_max_speed");
  if (max_speed_mhz > 0) {
    double max_freq_ghz = static_cast<double>(max_speed_mhz) / 1000.0;
    if (this->cpu_tdp_watts_ > 0 && this->cpu_cores_ > 0) {
      double calculated =
          this->cpu_tdp_watts_ / (this->cpu_cores_ * max_freq_ghz);
      return calculated;
    }
  }

  return FALLBACK_POWER_PER_CORE_PER_GHZ;
}

double PowerEstimator::estimate_watts_per_core() {
  double total_power_w = this->hwmon_scanner_.read_power_w();

  if (total_power_w == 0.0) {
    double battery_power = this->read_battery_power_w();
    if (battery_power > 0) {
      total_power_w = battery_power;
    }
  }

  if (total_power_w == 0.0) {
    total_power_w = this->read_rapl_power_limit_w();
  }

  if (total_power_w == 0.0) {
    long power_mw = SysfsReader::read_long(
        "/sys/class/thermal/thermal_zone0/sustainable_power");
    if (power_mw > 0) {
      total_power_w = static_cast<double>(power_mw) / 1000.0;
    }
  }

  if (total_power_w == 0.0 && this->cpu_tdp_watts_ > 0) {
    total_power_w = this->cpu_tdp_watts_;
  }

  if (total_power_w > 0 && this->cpu_cores_ > 0) {
    return total_power_w / this->cpu_cores_;
  }

  return FALLBACK_WATTS_PER_CORE;
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

          // Try ACPI max power
          if (tdp_watts == 0.0) {
            long power_uw = SysfsReader::read_long(
                "/sys/class/powercap/intel-rapl/intel-rapl:0/"
                "constraint_0_max_power_uw");
            if (power_uw > 0) {
              tdp_watts = static_cast<double>(power_uw) / 1e6;
            }
          }

          // Derive from CPU frequency
          if (tdp_watts == 0.0 && this->cpu_cores_ > 0) {
            long max_freq_khz = SysfsReader::read_long(
                "/sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_max_freq");
            if (max_freq_khz > 0) {
              double max_freq_ghz = static_cast<double>(max_freq_khz) / 1e6;
              double watts_per_ghz = this->read_watts_per_ghz();
              double watts_per_core_at_max = max_freq_ghz * watts_per_ghz;
              tdp_watts = this->cpu_cores_ * watts_per_core_at_max;
            }
          }

          break;
        }
      }
    }
  }

  closedir(thermal_dir);
  return tdp_watts;
}

double PowerEstimator::estimate_frequency_tdp_watts() {
  if (this->cpu_cores_ <= 0) {
    return 0.0;
  }

  long freq_khz = SysfsReader::read_long(
      "/sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_max_freq");
  double max_freq_ghz = static_cast<double>(freq_khz) / 1e6;

  if (max_freq_ghz > 0) {
    double power_factor = this->estimate_power_per_core_per_ghz();
    return this->cpu_cores_ * max_freq_ghz * power_factor;
  }

  return 0.0;
}

double PowerEstimator::estimate_cores_tdp_watts() {
  double watts_per_core = this->estimate_watts_per_core();
  return this->cpu_cores_ * watts_per_core;
}

} // namespace utils
} // namespace poirot
