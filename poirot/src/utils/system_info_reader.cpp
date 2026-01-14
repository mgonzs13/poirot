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

#include "poirot/utils/system_info_reader.hpp"

#include <fstream>
#include <set>
#include <sys/sysinfo.h>
#include <sys/utsname.h>
#include <thread>

namespace poirot {
namespace utils {

CpuInfo SystemInfoReader::read_cpu_info() {
  CpuInfo cpu_info;
  cpu_info.cores = 0;

  std::ifstream cpuinfo("/proc/cpuinfo");
  if (cpuinfo.is_open()) {
    std::string line;
    std::set<int> physical_cores;
    int current_physical_id = -1;
    int current_core_id = -1;

    while (std::getline(cpuinfo, line)) {
      if (line.find("model name") != std::string::npos) {
        size_t pos = line.find(':');
        if (pos != std::string::npos) {
          cpu_info.model = line.substr(pos + 2);
        }

      } else if (line.find("core id") != std::string::npos) {
        size_t pos = line.find(':');
        if (pos != std::string::npos) {
          current_core_id = std::stoi(line.substr(pos + 2));
          // Combine physical_id and core_id to get unique physical cores
          int unique_core = (current_physical_id << 16) | current_core_id;
          physical_cores.insert(unique_core);
        }
      }
    }

    cpu_info.cores = physical_cores.empty()
                         ? static_cast<int>(std::thread::hardware_concurrency())
                         : static_cast<int>(physical_cores.size());
  }

  if (cpu_info.cores == 0) {
    cpu_info.cores = static_cast<int>(std::thread::hardware_concurrency());
  }

  return cpu_info;
}

MemoryInfo SystemInfoReader::read_memory_info() {
  MemoryInfo mem_info;
  mem_info.total_kb = 0;

  struct sysinfo info;
  if (sysinfo(&info) == 0) {
    mem_info.total_kb = static_cast<long>(info.totalram * info.mem_unit / 1024);
  }

  return mem_info;
}

OsInfo SystemInfoReader::read_os_info() {
  OsInfo os_info;

  struct utsname uts;
  if (uname(&uts) == 0) {
    os_info.name = uts.sysname;
    os_info.version = uts.release;
    os_info.hostname = uts.nodename;
  }

  std::ifstream os_release("/etc/os-release");
  if (os_release.is_open()) {
    std::string line;
    while (std::getline(os_release, line)) {
      if (line.find("PRETTY_NAME=") == 0) {
        os_info.name = line.substr(13);
        if (!os_info.name.empty() && os_info.name.back() == '"') {
          os_info.name.pop_back();
        }
        break;
      }
    }
  }

  return os_info;
}

} // namespace utils
} // namespace poirot
