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
#include <thread>

#include "poirot/utils/process_metrics.hpp"

namespace poirot {
namespace utils {

ProcessMetrics::ProcessMetrics() {
  int num_cpus = static_cast<int>(std::thread::hardware_concurrency());
  this->num_cpus_ = (num_cpus > 0) ? num_cpus : 1;
}

int64_t ProcessMetrics::read_cpu_time_us() const {
  struct timespec ts;
  if (clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &ts) == 0) {
    return static_cast<int64_t>(
        std::round(static_cast<double>(ts.tv_sec) * 1e6 +
                   static_cast<double>(ts.tv_nsec) / 1e3));
  }
  return 0;
}

int ProcessMetrics::read_thread_count() const {
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

int64_t ProcessMetrics::read_memory_kb() const {
  std::ifstream file("/proc/self/status");
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

ProcessIoBytes ProcessMetrics::read_io_bytes() const {
  ProcessIoBytes io_bytes;

  std::ifstream file("/proc/self/io");
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

} // namespace utils
} // namespace poirot
