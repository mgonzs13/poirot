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

#include <syscall.h>

#include <fstream>
#include <sstream>

#include "poirot/utils/sysfs_reader.hpp"

namespace poirot {
namespace utils {

long SysfsReader::read_long(const std::string &path) {
  std::ifstream file(path);
  if (!file.is_open()) {
    return -1;
  }

  long value = -1;
  file >> value;
  return value;
}

double SysfsReader::read_double(const std::string &path) {
  std::ifstream file(path);
  if (!file.is_open()) {
    return 0.0;
  }

  double value = 0.0;
  file >> value;
  return value;
}

std::string SysfsReader::read_string(const std::string &path) {
  std::ifstream file(path);
  if (!file.is_open()) {
    return "";
  }

  std::string value;
  std::getline(file, value);
  return value;
}

std::string SysfsReader::get_thread_status_path(const std::string &filename) {
  pid_t tid = static_cast<pid_t>(syscall(SYS_gettid));
  std::ostringstream oss;
  oss << "/proc/self/task/" << tid << "/" << filename;

  // Check if thread-specific path exists
  std::ifstream test(oss.str());
  if (test.is_open()) {
    return oss.str();
  }

  // Fall back to process-level path
  return "/proc/self/" + filename;
}

} // namespace utils
} // namespace poirot
