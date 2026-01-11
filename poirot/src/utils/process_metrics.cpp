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

double ProcessMetrics::read_cpu_time_us() {
  struct timespec ts;
  if (clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &ts) == 0) {
    return static_cast<double>(ts.tv_sec) * 1e6 +
           static_cast<double>(ts.tv_nsec) / 1e3;
  }
  return 0.0;
}

double ProcessMetrics::read_total_cpu_time_us() {
  struct timespec ts;
  if (clock_gettime(CLOCK_MONOTONIC, &ts) == 0) {
    double wall_time_us = static_cast<double>(ts.tv_sec) * 1e6 +
                          static_cast<double>(ts.tv_nsec) / 1e3;
    return wall_time_us * static_cast<double>(this->num_cpus_);
  }
  return 0.0;
}

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

  std::ifstream stat("/proc/self/stat");
  if (!stat.is_open()) {
    return 0.0;
  }

  std::string line;
  std::getline(stat, line);

  size_t comm_start = line.find('(');
  size_t comm_end = line.rfind(')');
  if (comm_start == std::string::npos || comm_end == std::string::npos) {
    return 0.0;
  }

  std::string after_comm = line.substr(comm_end + 2);
  std::istringstream iss(after_comm);
  std::string token;

  unsigned long long utime = 0;
  unsigned long long stime = 0;

  for (int i = 1; i <= 13; ++i) {
    if (!(iss >> token)) {
      return 0.0;
    }

    if (i == 12) {
      try {
        utime = std::stoull(token);
      } catch (...) {
        return 0.0;
      }
    } else if (i == 13) {
      try {
        stime = std::stoull(token);
      } catch (...) {
        return 0.0;
      }
    }
  }

  auto now = std::chrono::high_resolution_clock::now();
  double time_diff =
      std::chrono::duration<double>(now - this->prev_cpu_read_time_).count();

  double pct = 0.0;
  unsigned long long prev_cpu = this->prev_process_cpu_.load();

  if (time_diff > 0 && prev_cpu > 0) {
    unsigned long long cpu_diff = (utime + stime) - prev_cpu;
    long sc_clk_tck = sysconf(_SC_CLK_TCK);
    double cpu_seconds =
        static_cast<double>(cpu_diff) / static_cast<double>(sc_clk_tck);
    pct = (cpu_seconds / time_diff) * 100.0;
  }

  this->prev_process_cpu_.store(utime + stime);
  this->prev_cpu_read_time_ = now;
  return pct;
}

} // namespace utils
} // namespace poirot
