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

#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>

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
  return this->read_energy_uj_locked();
}

double RaplMonitor::read_energy_uj_locked() {
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

double RaplMonitor::read_system_busy_us_locked() const {
  std::ifstream file("/proc/stat");
  if (!file.is_open()) {
    return 0.0;
  }

  std::string line;
  if (!std::getline(file, line)) {
    return 0.0;
  }

  long clk_tck = sysconf(_SC_CLK_TCK);
  return this->parse_proc_stat_busy_us(line, clk_tck);
}

EnergySnapshot RaplMonitor::read_snapshot() {
  std::lock_guard<std::mutex> lock(this->rapl_mutex_);

  double energy_uj = this->read_energy_uj_locked();

  EnergySnapshot snapshot;
  snapshot.energy_uj = std::max(0.0, energy_uj);
  snapshot.busy_us = this->read_system_busy_us_locked();
  snapshot.valid = energy_uj >= 0.0;

  if (snapshot.valid) {
    int64_t now_us = std::chrono::duration_cast<std::chrono::microseconds>(
                         std::chrono::steady_clock::now().time_since_epoch())
                         .count();
    this->rate_window_.update(now_us, snapshot.energy_uj, snapshot.busy_us,
                              RATE_WINDOW_US);
  }

  return snapshot;
}

double RaplMonitor::calculate_call_energy_uj(const EnergySnapshot &start,
                                             const EnergySnapshot &end,
                                             double thread_cpu_delta_us) {
  if (!start.valid || !end.valid) {
    return 0.0;
  }

  double delta_energy_uj = end.energy_uj - start.energy_uj;
  double delta_busy_us = end.busy_us - start.busy_us;

  std::lock_guard<std::mutex> lock(this->rapl_mutex_);
  return this->attribute_energy(delta_energy_uj, delta_busy_us,
                                thread_cpu_delta_us,
                                this->rate_window_.rate_uj_per_busy_us);
}

void RateWindow::update(int64_t now_us, double energy_uj, double busy_us,
                        int64_t window_us) {
  if (this->start_time_us == 0) {
    this->start_time_us = now_us;
    this->start_energy_uj = energy_uj;
    this->start_busy_us = busy_us;
    return;
  }

  if (now_us - this->start_time_us < window_us) {
    return;
  }

  double delta_energy_uj = energy_uj - this->start_energy_uj;
  double delta_busy_us = busy_us - this->start_busy_us;
  if (delta_energy_uj > 0.0 && delta_busy_us > 0.0) {
    this->rate_uj_per_busy_us = delta_energy_uj / delta_busy_us;
  }

  this->start_time_us = now_us;
  this->start_energy_uj = energy_uj;
  this->start_busy_us = busy_us;
}

double RaplMonitor::parse_proc_stat_busy_us(const std::string &line,
                                            long clk_tck) {
  std::istringstream iss(line);
  std::string label;
  if (!(iss >> label) || label != "cpu") {
    return 0.0;
  }

  // Fields: user nice system idle iowait irq softirq steal ...
  constexpr int STAT_FIELDS = 8;
  double values[STAT_FIELDS] = {0.0};
  for (int i = 0; i < STAT_FIELDS; ++i) {
    if (!(iss >> values[i])) {
      return 0.0;
    }
  }

  if (clk_tck <= 0) {
    clk_tck = 100;
  }

  double busy_jiffies =
      values[0] + values[1] + values[2] + values[5] + values[6] + values[7];
  return busy_jiffies * 1e6 / static_cast<double>(clk_tck);
}

double RaplMonitor::attribute_energy(double delta_energy_uj,
                                     double delta_busy_us,
                                     double thread_cpu_delta_us,
                                     double rate_uj_per_busy_us) {
  if (thread_cpu_delta_us <= 0.0) {
    return 0.0;
  }

  if (delta_energy_uj > 0.0 && delta_busy_us >= MIN_INTERVAL_US) {
    double share = thread_cpu_delta_us / delta_busy_us;
    share = std::max(0.0, std::min(1.0, share));
    return delta_energy_uj * share;
  }

  if (rate_uj_per_busy_us > 0.0) {
    return rate_uj_per_busy_us * thread_cpu_delta_us;
  }

  return 0.0;
}

} // namespace utils
} // namespace poirot
