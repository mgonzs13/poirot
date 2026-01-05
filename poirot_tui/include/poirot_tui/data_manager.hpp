// Copyright 2025 Miguel Ángel González Santamarta
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

#ifndef POIROT_TUI__DATA_MANAGER_HPP_
#define POIROT_TUI__DATA_MANAGER_HPP_

#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include "poirot_msgs/msg/profiling_data.hpp"

namespace poirot_tui {

/// Represents a single data point for graphing
struct DataPoint {
  double timestamp;
  double wall_time_us;
  double cpu_time_us;
  int64_t memory_kb;
  int64_t io_read_bytes;
  int64_t io_write_bytes;
  int64_t ctx_switches;
  double energy_uj;
  double co2_ug;
};

/// Represents a function's profiling data row
struct FunctionRow {
  int32_t pid;
  std::string node_name;
  std::string function_name;
  int32_t call_count;
  double wall_time_us;
  double cpu_time_us;
  int64_t memory_kb;
  int64_t io_read_bytes;
  int64_t io_write_bytes;
  int64_t ctx_switches;
  double energy_uj;
  double co2_ug;
  double last_update_time;

  /// Unique key for identifying this function
  std::string getKey() const {
    return std::to_string(pid) + "|" + node_name + "|" + function_name;
  }
};

/// Sort column enumeration
enum class SortColumn {
  PID,
  NODE_NAME,
  FUNCTION_NAME,
  CALL_COUNT,
  WALL_TIME,
  CPU_TIME,
  MEMORY,
  IO_READ,
  IO_WRITE,
  CTX_SWITCHES,
  ENERGY,
  CO2
};

/// Graph data type enumeration
enum class GraphDataType {
  WALL_TIME,
  CPU_TIME,
  MEMORY,
  IO_READ,
  IO_WRITE,
  CTX_SWITCHES,
  ENERGY,
  CO2
};

/// Manages profiling data storage and retrieval
class DataManager {
public:
  static constexpr size_t MAX_HISTORY_SIZE = 1000;

  DataManager();
  ~DataManager() = default;

  /// Process incoming profiling data
  void
  processProfilingData(const poirot_msgs::msg::ProfilingData::SharedPtr msg);

  /// Get sorted function rows
  std::vector<FunctionRow> getSortedRows(SortColumn column,
                                         bool ascending) const;

  /// Get historical data for a function
  std::vector<DataPoint> getHistory(const std::string &function_key) const;

  /// Get all function keys
  std::vector<std::string> getAllFunctionKeys() const;

  /// Check if a function is enabled for graphing
  bool isFunctionEnabled(const std::string &function_key) const;

  /// Enable a function for graphing
  void enableFunction(const std::string &function_key);

  /// Disable a function for graphing
  void disableFunction(const std::string &function_key);

  /// Toggle function graph visibility
  void toggleFunction(const std::string &function_key);

  /// Get enabled functions
  std::set<std::string> getEnabledFunctions() const;

  /// Clear all data
  void clear();

  /// Get total number of functions
  size_t getFunctionCount() const;

private:
  mutable std::mutex mutex_;
  std::map<std::string, FunctionRow> function_rows_;
  std::map<std::string, std::deque<DataPoint>> function_history_;
  std::set<std::string> enabled_functions_;
  double start_time_;
  bool first_message_;
};

} // namespace poirot_tui

#endif // POIROT_TUI__DATA_MANAGER_HPP_
