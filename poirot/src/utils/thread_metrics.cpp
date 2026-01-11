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

#include <cmath>
#include <fstream>
#include <sstream>
#include <time.h>

#include "poirot/utils/sysfs_reader.hpp"
#include "poirot/utils/thread_metrics.hpp"

namespace poirot {
namespace utils {

long ThreadMetrics::read_cpu_time_us() {
  struct timespec ts;
  if (clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts) == 0) {
    return std::round(static_cast<double>(ts.tv_sec) * 1e6 +
                      static_cast<double>(ts.tv_nsec) / 1e3);
  }
  return 0.0;
}

long ThreadMetrics::read_memory_kb() {
  std::string status_path = SysfsReader::get_thread_status_path("status");
  std::ifstream file(status_path);
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

void ThreadMetrics::read_io_bytes(long &read_bytes, long &write_bytes) {
  read_bytes = 0;
  write_bytes = 0;

  std::string io_path = SysfsReader::get_thread_status_path("io");
  std::ifstream file(io_path);
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

long ThreadMetrics::read_ctx_switches() {
  std::string status_path = SysfsReader::get_thread_status_path("status");
  std::ifstream file(status_path);
  if (!file.is_open()) {
    return 0;
  }

  std::string line;
  long voluntary = 0;
  long nonvoluntary = 0;

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
