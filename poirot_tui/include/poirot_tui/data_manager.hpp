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

/**
 * @brief Represents a single data point in function history
 */

struct DataPoint {
  double timestamp;
  int64_t wall_time_us;
  int64_t cpu_time_us;
  int64_t mem_kb;
  int64_t gpu_mem_kb;
  int64_t io_read_bytes;
  int64_t io_write_bytes;
  int64_t ctx_switches;
  double cpu_energy_uj;
  double gpu_energy_uj;
  double energy_uj;
  double co2_ug;
};

/**
 * @brief Represents a row of function profiling data
 */
struct FunctionRow {
  int32_t pid;
  std::string function_name;
  int32_t call_count;
  int64_t wall_time_us;
  int64_t cpu_time_us;
  int64_t mem_kb;
  int64_t gpu_mem_kb;
  int64_t io_read_bytes;
  int64_t io_write_bytes;
  int64_t ctx_switches;
  double cpu_energy_uj;
  double gpu_energy_uj;
  double energy_uj;
  double co2_ug;
  double last_update_time;

  /// @brief Unique key for identifying this function
  std::string get_key() const {
    return std::to_string(pid) + "|" + function_name;
  }
};

/**
 * @brief Sorting column enumeration
 */
enum class SortColumn {
  PID,
  FUNCTION_NAME,
  CALL_COUNT,
  WALL_TIME,
  CPU_TIME,
  MEMORY,
  GPU_MEMORY,
  IO_READ,
  IO_WRITE,
  CTX_SWITCHES,
  CPU_ENERGY,
  GPU_ENERGY,
  ENERGY,
  CO2
};

/**
 * @brief Graph data type enumeration
 */
enum class GraphDataType {
  WALL_TIME,
  CPU_TIME,
  MEMORY,
  GPU_MEMORY,
  IO_READ,
  IO_WRITE,
  CTX_SWITCHES,
  CPU_ENERGY,
  GPU_ENERGY,
  ENERGY,
  CO2
};

/**
 * @class DataManager
 * @brief Manages profiling data for the TUI
 */
class DataManager {
public:
  /// @brief Maximum number of historical data points to store per function
  static constexpr size_t MAX_HISTORY_SIZE = 1000;

  /**
   * @brief Constructor for DataManager
   */
  DataManager();

  /**
   * @brief Destructor for DataManager
   */
  ~DataManager() = default;

  /**
   * @brief Process incoming profiling data message
   * @param msg Shared pointer to the profiling data message
   */
  void
  process_profiling_data(const poirot_msgs::msg::ProfilingData::SharedPtr msg);

  /**
   * @brief Get sorted function rows
   * @param column Column to sort by
   * @param ascending Sort order
   * @return Vector of sorted FunctionRows
   */
  std::vector<FunctionRow> get_sorted_rows(SortColumn column,
                                           bool ascending) const;

  /**
   * @brief Get historical data for a specific function
   * @param function_key Unique key of the function
   * @return Vector of DataPoints for the function
   */
  std::vector<DataPoint> get_history(const std::string &function_key) const;

  /**
   * @brief Get all function keys
   * @return Vector of all function keys
   */
  std::vector<std::string> get_all_function_keys() const;

  /**
   * @brief Check if a function is enabled for graphing
   * @param function_key Unique key of the function
   * @return True if enabled, false otherwise
   */
  bool is_function_enabled(const std::string &function_key) const;

  /**
   * @brief Enable a function for graphing
   * @param function_key Unique key of the function
   */
  void enable_function(const std::string &function_key);

  /**
   * @brief Disable a function for graphing
   * @param function_key Unique key of the function
   */
  void disable_function(const std::string &function_key);

  /**
   * @brief Toggle function's enabled state for graphing
   * @param function_key Unique key of the function
   */
  void toggle_function(const std::string &function_key);

  /**
   * @brief Get all enabled functions
   * @return Set of enabled function keys
   */
  std::set<std::string> get_enabled_functions() const;

  /**
   * @brief Clear all stored data
   */
  void clear();

  /**
   * @brief Get the count of tracked functions
   * @return Number of tracked functions
   */
  size_t get_function_count() const;

private:
  /// @brief Mutex for thread-safe access
  mutable std::mutex mutex_;
  /// @brief Map of function keys to their profiling data rows
  std::map<std::string, FunctionRow> function_rows_;
  /// @brief Map of function keys to their historical data points
  std::map<std::string, std::deque<DataPoint>> function_history_;
  /// @brief Set of enabled function keys for graphing
  std::set<std::string> enabled_functions_;
  /// @brief Start time for relative timestamp calculations
  double start_time_;
  /// @brief Flag to indicate if the first message has been processed
  bool first_message_;
};

} // namespace poirot_tui

#endif // POIROT_TUI__DATA_MANAGER_HPP_
