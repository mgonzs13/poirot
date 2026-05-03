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

  // Record the wall-clock time on the very first successful RAPL read.
  // This is used to compute the long-running average power.
  if (this->start_time_.time_since_epoch().count() == 0) {
    this->start_time_ = std::chrono::steady_clock::now();
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

double RaplMonitor::get_average_power_uj_per_us() {
  std::lock_guard<std::mutex> lock(this->rapl_mutex_);

  if (this->start_time_.time_since_epoch().count() == 0 ||
      this->accumulated_energy_uj_ <= 0.0) {
    return 0.0;
  }

  auto now = std::chrono::steady_clock::now();
  double elapsed_us =
      static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
                              now - this->start_time_)
                              .count());
  if (elapsed_us <= 0.0) {
    return 0.0;
  }

  return this->accumulated_energy_uj_ / elapsed_us;
}

double RaplMonitor::calculate_thread_energy_uj(double thread_cpu_delta_us) {
  if (thread_cpu_delta_us <= 0.0) {
    return 0.0;
  }

  // Use the long-running average RAPL power (total accumulated energy /
  // total elapsed wall time since first read).  Multiplied by the thread's
  // own CPU time (CLOCK_THREAD_CPUTIME_ID delta), this gives the energy
  // attributed to this thread's actual compute work.
  //
  // Inclusivity proof: CLOCK_THREAD_CPUTIME_ID is a per-thread cumulative
  // monotonic counter.  For any nested call A→B on the same thread,
  // thread_cpu_delta_A >= thread_cpu_delta_B always.  Since avg_power is the
  // same constant for both, energy_A >= energy_B holds unconditionally —
  // no clamping or child-accumulation is needed.
  double avg_power_uj_per_us = this->get_average_power_uj_per_us();
  if (avg_power_uj_per_us <= 0.0) {
    return 0.0;
  }

  return avg_power_uj_per_us * thread_cpu_delta_us;
}

} // namespace utils
} // namespace poirot
