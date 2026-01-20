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

#include "poirot/utils/sysfs_reader.hpp"
#include "poirot/utils/thread_metrics.hpp"

namespace poirot {
namespace utils {

int64_t ThreadMetrics::read_cpu_time_us() const {
  struct timespec ts;
  if (clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts) == 0) {
    return static_cast<int64_t>(
        std::round(static_cast<double>(ts.tv_sec) * 1e6 +
                   static_cast<double>(ts.tv_nsec) / 1e3));
  }
  return 0;
}

int64_t ThreadMetrics::read_process_cpu_time_us() const {
  struct timespec ts;
  if (clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &ts) == 0) {
    return static_cast<int64_t>(
        std::round(static_cast<double>(ts.tv_sec) * 1e6 +
                   static_cast<double>(ts.tv_nsec) / 1e3));
  }
  return 0;
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

int ThreadMetrics::get_pid() const { return static_cast<int>(getpid()); }

int ThreadMetrics::read_num_threads() const {
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

} // namespace utils
} // namespace poirot
