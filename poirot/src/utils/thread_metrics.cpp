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

#include <time.h>

#include <cmath>
#include <fstream>
#include <sstream>
#include <thread>

#include "poirot/utils/sysfs_reader.hpp"
#include "poirot/utils/thread_metrics.hpp"

namespace poirot {
namespace utils {

ThreadMetrics::ThreadMetrics() {
  int num_cpus = static_cast<int>(std::thread::hardware_concurrency());
  this->num_cpus_ = (num_cpus > 0) ? num_cpus : 1;
}

int64_t ThreadMetrics::read_cpu_time_us() const {
  struct timespec ts;
  if (clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts) == 0) {
    return static_cast<int64_t>(
        std::round(static_cast<double>(ts.tv_sec) * 1e6 +
                   static_cast<double>(ts.tv_nsec) / 1e3));
  }
  return 0;
}

double ThreadMetrics::read_cpu_percent() {
  std::lock_guard<std::mutex> lock(this->cpu_read_mutex_);

  // Read current thread CPU time
  int64_t current_thread_cpu_us = this->read_cpu_time_us();

  if (current_thread_cpu_us <= 0) {
    return 0.0;
  }

  // First call - initialize state and return 0
  uint64_t prev_thread = this->prev_thread_cpu_us_.load();
  if (prev_thread == 0) {
    this->prev_thread_cpu_us_.store(
        static_cast<uint64_t>(current_thread_cpu_us));
    this->prev_cpu_read_time_ = std::chrono::steady_clock::now();
    return 0.0;
  }

  // Calculate time elapsed since last measurement
  auto now = std::chrono::steady_clock::now();
  double elapsed_us =
      static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
                              now - this->prev_cpu_read_time_)
                              .count());

  if (elapsed_us <= 0.0) {
    return 0.0;
  }

  // Calculate CPU time delta for thread
  double thread_cpu_delta_us = static_cast<double>(current_thread_cpu_us) -
                               static_cast<double>(prev_thread);

  // Total available CPU time in this period
  double total_available_cpu_us =
      elapsed_us * static_cast<double>(this->num_cpus_);

  // Calculate percentage
  double pct = 0.0;
  if (total_available_cpu_us > 0.0) {
    pct = (thread_cpu_delta_us / total_available_cpu_us) * 100.0;
  }

  // Update state for next measurement
  this->prev_thread_cpu_us_.store(static_cast<uint64_t>(current_thread_cpu_us));
  this->prev_cpu_read_time_ = now;

  return pct;
}

int64_t ThreadMetrics::read_memory_kb() const {
  std::string status_path = SysfsReader::get_thread_status_path("status");
  std::ifstream file(status_path);
  if (!file.is_open()) {
    return 0;
  }

  std::string line;
  while (std::getline(file, line)) {
    if (line.compare(0, 6, "VmRSS:") == 0) {
      std::istringstream iss(line.substr(6));
      int64_t value = 0;
      iss >> value;
      return value;
    }
  }

  return 0;
}

ThreadIoBytes ThreadMetrics::read_io_bytes() const {
  ThreadIoBytes io_bytes;

  std::string io_path = SysfsReader::get_thread_status_path("io");
  std::ifstream file(io_path);
  if (!file.is_open()) {
    return io_bytes;
  }

  std::string line;
  while (std::getline(file, line)) {
    if (line.compare(0, 11, "read_bytes:") == 0) {
      io_bytes.read_bytes = std::stoll(line.substr(11));
    } else if (line.compare(0, 12, "write_bytes:") == 0) {
      io_bytes.write_bytes = std::stoll(line.substr(12));
    }
  }

  return io_bytes;
}

int64_t ThreadMetrics::read_context_switches() const {
  std::string status_path = SysfsReader::get_thread_status_path("status");
  std::ifstream file(status_path);
  if (!file.is_open()) {
    return 0;
  }

  std::string line;
  int64_t voluntary = 0;
  int64_t nonvoluntary = 0;

  while (std::getline(file, line)) {
    if (line.compare(0, 24, "voluntary_ctxt_switches:") == 0) {
      std::istringstream iss(line.substr(24));
      iss >> voluntary;
    } else if (line.compare(0, 27, "nonvoluntary_ctxt_switches:") == 0) {
      std::istringstream iss(line.substr(27));
      iss >> nonvoluntary;
    }
  }

  return voluntary + nonvoluntary;
}

} // namespace utils
} // namespace poirot
