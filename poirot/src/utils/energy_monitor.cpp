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

#include "poirot/utils/energy_monitor.hpp"
#include "poirot/utils/sysfs_reader.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>

namespace poirot {
namespace utils {

namespace fs = std::filesystem;

EnergyMonitor::EnergyMonitor(HwmonScanner &hwmon_scanner)
    : hwmon_scanner_(hwmon_scanner),
      last_energy_read_time_(std::chrono::high_resolution_clock::now()) {}

void EnergyMonitor::initialize_rapl_max_energy() {
  // Get max energy range for wraparound detection
  const std::string max_range_path =
      "/sys/class/powercap/intel-rapl:0/max_energy_range_uj";

  if (fs::exists(max_range_path)) {
    double max_range = SysfsReader::read_double(max_range_path);
    if (max_range > 0) {
      this->rapl_max_energy_uj_ = max_range;
    }
  }
}

double EnergyMonitor::read_energy_uj(double cpu_percent_) {
  std::lock_guard<std::mutex> lock(this->energy_mutex_);

  // Helper lambda to read RAPL energy with wraparound detection
  auto read_rapl_energy = [this](const std::string &path) -> double {
    if (!fs::exists(path)) {
      return -1.0;
    }

    double current_energy = SysfsReader::read_double(path);
    if (current_energy <= 0) {
      return -1.0;
    }

    // Handle RAPL counter wraparound
    if (this->last_rapl_energy_uj_ > 0) {
      if (current_energy < this->last_rapl_energy_uj_) {
        // Counter wrapped around
        if (this->rapl_max_energy_uj_ > 0) {
          double delta =
              (this->rapl_max_energy_uj_ - this->last_rapl_energy_uj_) +
              current_energy;
          this->accumulated_energy_uj_ += delta;
        }
      } else {
        this->accumulated_energy_uj_ +=
            (current_energy - this->last_rapl_energy_uj_);
      }
    }

    this->last_rapl_energy_uj_ = current_energy;
    return this->accumulated_energy_uj_;
  };

  // 1. Try Intel RAPL (most accurate for Intel CPUs)
  double energy =
      read_rapl_energy("/sys/class/powercap/intel-rapl:0/energy_uj");
  if (energy >= 0) {
    return energy;
  }

  // 2. Try AMD RAPL
  energy = read_rapl_energy("/sys/class/powercap/amd-rapl:0/energy_uj");
  if (energy >= 0) {
    return energy;
  }

  // 3. Try hwmon energy path
  const std::string &hwmon_energy_path = this->hwmon_scanner_.get_energy_path();
  if (!hwmon_energy_path.empty() && fs::exists(hwmon_energy_path)) {
    double current_hwmon_energy = SysfsReader::read_double(hwmon_energy_path);

    if (current_hwmon_energy > 0) {
      if (this->last_hwmon_energy_uj_ > 0) {
        if (current_hwmon_energy >= this->last_hwmon_energy_uj_) {
          // Normal case: accumulate the delta
          this->accumulated_energy_uj_ +=
              (current_hwmon_energy - this->last_hwmon_energy_uj_);
        } else {
          // Counter wraparound detected; ignore this reading
        }
      }

      this->last_hwmon_energy_uj_ = current_hwmon_energy;
      return this->accumulated_energy_uj_;
    }
  }

  // 4. Estimate energy based on power measurements and elapsed time
  auto now = std::chrono::high_resolution_clock::now();

  if (this->last_energy_read_time_.time_since_epoch().count() == 0) {
    this->last_energy_read_time_ = now;
    return this->accumulated_energy_uj_;
  }

  double elapsed_us =
      static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
                              now - this->last_energy_read_time_)
                              .count());
  this->last_energy_read_time_ = now;

  if (elapsed_us <= 0) {
    return this->accumulated_energy_uj_;
  }

  // Try to get actual power measurement
  double power_w = this->hwmon_scanner_.read_power_w();

  // Estimate power if no direct reading available
  if (power_w <= 0.0 && this->cpu_tdp_watts_ > 0.0) {
    // Estimate based on CPU utilization
    double base_power = this->cpu_tdp_watts_ * this->idle_power_factor_;
    double dynamic_power = this->cpu_tdp_watts_ *
                           (1.0 - this->idle_power_factor_) *
                           (cpu_percent_ / 100.0);
    power_w = base_power + dynamic_power;
  }

  if (power_w > 0.0) {
    // Convert power and elapsed time to energy (microjoules)
    double energy_delta_uj = power_w * elapsed_us;
    this->accumulated_energy_uj_ += energy_delta_uj;
  }

  return this->accumulated_energy_uj_;
}

double EnergyMonitor::calculate_thread_energy_uj(double thread_cpu_delta_us,
                                                 double process_cpu_delta_us,
                                                 double system_cpu_delta_us,
                                                 double total_energy_delta_uj) {
  // If no energy delta, return 0
  if (total_energy_delta_uj <= 0.0) {
    return 0.0;
  }

  // If no system CPU delta, fallback to process-level attribution
  if (system_cpu_delta_us <= 0.0) {
    if (thread_cpu_delta_us > 0.0 && process_cpu_delta_us > 0.0) {
      return total_energy_delta_uj *
             (thread_cpu_delta_us / process_cpu_delta_us);
    }

    if (thread_cpu_delta_us > 0.0) {
      return total_energy_delta_uj;
    }

    return 0.0;
  }

  // Calculate thread's share of system CPU time
  double thread_share = 0.0;

  if (process_cpu_delta_us > 0.0 && thread_cpu_delta_us > 0.0) {
    // Thread's share of process time * process's share of system time
    double thread_process_share = thread_cpu_delta_us / process_cpu_delta_us;
    double process_system_share = process_cpu_delta_us / system_cpu_delta_us;
    thread_share = thread_process_share * process_system_share;
  } else if (thread_cpu_delta_us > 0.0) {
    // Direct thread to system ratio
    thread_share = thread_cpu_delta_us / system_cpu_delta_us;
  }

  // Clamp share to valid range
  thread_share = std::max(0.0, std::min(1.0, thread_share));

  return total_energy_delta_uj * thread_share;
}

} // namespace utils
} // namespace poirot
