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

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>

#include "poirot/utils/rapl_monitor.hpp"
#include "poirot/utils/sysfs_reader.hpp"

namespace poirot {
namespace utils {

namespace fs = std::filesystem;

RaplMonitor::RaplMonitor() {
  this->load_rapl_package_path();
  this->initialize_rapl_max_energy();
}

void RaplMonitor::load_rapl_package_path() {
  std::lock_guard<std::mutex> lock(this->rapl_mutex_);

  if (!fs::exists(RAPL_PATH)) {
    return;
  }

  for (const auto &entry : fs::directory_iterator(RAPL_PATH)) {
    const std::string &domain_path = entry.path().string();
    const std::string name_path = domain_path + "/name";

    if (fs::exists(name_path)) {
      std::string domain_name = SysfsReader::read_string(name_path);

      if (domain_name.find("package") != std::string::npos) {
        this->rapl_package_path_ = domain_path;
        break;
      }
    }
  }
}

void RaplMonitor::initialize_rapl_max_energy() {
  std::lock_guard<std::mutex> lock(this->rapl_mutex_);

  const std::string path = this->rapl_package_path_ + "/max_energy_range_uj";

  if (!fs::exists(path)) {
    this->rapl_max_energy_uj_ = 0.0;
    return;
  }

  double max_energy = SysfsReader::read_double(path);
  if (max_energy <= 0) {
    this->rapl_max_energy_uj_ = 0.0;
    return;
  }

  this->rapl_max_energy_uj_ = max_energy;
}

double RaplMonitor::read_energy_uj() {
  std::lock_guard<std::mutex> lock(this->rapl_mutex_);

  const std::string path = this->rapl_package_path_ + "/energy_uj";

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
}

double RaplMonitor::calculate_thread_energy_uj(double thread_cpu_delta_us,
                                               double process_cpu_delta_us,
                                               double system_cpu_delta_us,
                                               double total_energy_delta_uj) {
  // If no energy delta, return 0
  if (total_energy_delta_uj <= 0.0) {
    return 0.0;
  }

  // If no thread CPU time, return 0
  if (thread_cpu_delta_us <= 0.0) {
    return 0.0;
  }

  // Calculate thread's share of system energy using hierarchical attribution
  double thread_share = 0.0;

  if (system_cpu_delta_us > 0.0 && process_cpu_delta_us > 0.0) {
    // Hierarchical attribution:
    // thread_share = (thread_cpu / process_cpu) * (process_cpu / system_cpu)
    //              = thread_cpu / system_cpu
    // But we use the hierarchical form to be more accurate when there are
    // multiple processes and threads
    double thread_process_share = thread_cpu_delta_us / process_cpu_delta_us;
    double process_system_share = process_cpu_delta_us / system_cpu_delta_us;
    thread_share = thread_process_share * process_system_share;
  } else if (process_cpu_delta_us > 0.0) {
    // Fallback: attribute based on thread's share of process time
    thread_share = thread_cpu_delta_us / process_cpu_delta_us;
  } else {
    // Last resort: assume thread gets all the energy
    thread_share = 1.0;
  }

  // Clamp share to valid range [0, 1]
  thread_share = std::max(0.0, std::min(1.0, thread_share));

  return total_energy_delta_uj * thread_share;
}

} // namespace utils
} // namespace poirot
