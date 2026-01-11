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
#include <unistd.h>

#include <cmath>
#include <fstream>
#include <sstream>

#include "poirot/utils/process_metrics.hpp"

namespace poirot {
namespace utils {

ProcessMetrics::ProcessMetrics() {
  int num_cpus = static_cast<int>(std::thread::hardware_concurrency());
  this->num_cpus_ = (num_cpus > 0) ? num_cpus : 1;
}

ProcessMetrics::~ProcessMetrics() {}

long ProcessMetrics::read_cpu_time_us() {
  struct timespec ts;
  if (clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &ts) == 0) {
    return std::round(static_cast<double>(ts.tv_sec) * 1e6 +
                      static_cast<double>(ts.tv_nsec) / 1e3);
  }
  return 0.0;
}

int ProcessMetrics::get_num_cpus() const { return this->num_cpus_; }

int ProcessMetrics::read_thread_count() {
  std::ifstream status("/proc/self/status");
  if (!status.is_open()) {
    return 1;
  }

  std::string line;
  while (std::getline(status, line)) {
    if (line.compare(0, 8, "Threads:") == 0) {
      std::istringstream iss(line.substr(8));
      int threads = 1;
      iss >> threads;
      return threads;
    }
  }

  return 1;
}

double ProcessMetrics::read_cpu_percent() {
  std::lock_guard<std::mutex> lock(this->cpu_read_mutex_);

  // Read current process CPU time
  double current_process_cpu_us = this->read_cpu_time_us();

  if (current_process_cpu_us <= 0.0) {
    return 0.0;
  }

  // First call - initialize state and return 0
  double prev_process = this->prev_process_cpu_.load();
  if (prev_process == 0.0) {
    // Store as integer microseconds to fit in atomic
    this->prev_process_cpu_.store(
        static_cast<unsigned long long>(current_process_cpu_us));
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

  // Calculate CPU time delta for process
  double process_cpu_delta_us = current_process_cpu_us - prev_process;

  // Total available CPU time in this period
  double total_available_cpu_us =
      elapsed_us * static_cast<double>(this->num_cpus_);

  // Calculate percentage
  double pct = 0.0;
  if (total_available_cpu_us > 0.0) {
    pct = (process_cpu_delta_us / total_available_cpu_us) * 100.0;
  }

  // Update state for next measurement
  this->prev_process_cpu_.store(
      static_cast<unsigned long long>(current_process_cpu_us));
  this->prev_cpu_read_time_ = now;

  return pct;
}

long ProcessMetrics::read_memory_kb() {
  std::ifstream file("/proc/self/status");
  if (!file.is_open()) {
    return 0;
  }

  std::string line;
  while (std::getline(file, line)) {
    if (line.compare(0, 6, "VmRSS:") == 0) {
      std::istringstream iss(line.substr(6));
      long value = 0;
      iss >> value;
      return value;
    }
  }

  return 0;
}

void ProcessMetrics::read_io_bytes(long &read_bytes, long &write_bytes) {
  read_bytes = 0;
  write_bytes = 0;

  std::ifstream file("/proc/self/io");
  if (!file.is_open()) {
    return;
  }

  std::string line;
  while (std::getline(file, line)) {
    if (line.compare(0, 11, "read_bytes:") == 0) {
      read_bytes = std::stol(line.substr(11));
    } else if (line.compare(0, 12, "write_bytes:") == 0) {
      write_bytes = std::stol(line.substr(12));
    }
  }
}

} // namespace utils
} // namespace poirot
