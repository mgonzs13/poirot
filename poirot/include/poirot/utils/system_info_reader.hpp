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

#ifndef POIROT__UTILS__SYSTEM_INFO_READER_HPP_
#define POIROT__UTILS__SYSTEM_INFO_READER_HPP_

#include <string>

namespace poirot {
namespace utils {

/// @brief Structure to hold CPU information
struct CpuInfo {
  std::string model;
  int cores;
};

/// @brief Structure to hold Memory information
struct MemoryInfo {
  long total_kb;
};

/// @brief Structure to hold OS information
struct OsInfo {
  std::string name;
  std::string version;
  std::string hostname;
};

/**
 * @class SystemInfoReader
 * @brief Class for reading system information.
 * Provides methods to read CPU, memory, and OS information.
 */
class SystemInfoReader {
public:
  /**
   * @brief Default constructor and destructor.
   */
  SystemInfoReader() = default;

  /**
   * @brief Default destructor.
   */
  ~SystemInfoReader() = default;

  /**
   * @brief Read CPU information from /proc/cpuinfo
   * @return CpuInfo struct containing model and core count
   */
  CpuInfo read_cpu_info();

  /**
   * @brief Read total system memory
   * @return MemoryInfo struct containing total memory in KB
   */
  MemoryInfo read_memory_info();

  /**
   * @brief Read OS information from uname and /etc/os-release
   * @return OsInfo struct containing OS name, version, and hostname
   */
  OsInfo read_os_info();
};

} // namespace utils
} // namespace poirot

#endif // POIROT_UTILS_SYSTEM_INFO_READER_HPP
