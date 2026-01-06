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

#ifndef POIROT__POIROT_HPP_
#define POIROT__POIROT_HPP_

#include <sys/resource.h>
#include <sys/sysinfo.h>
#include <sys/utsname.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "poirot_msgs/msg/function_stats.hpp"
#include "poirot_msgs/msg/process_info.hpp"
#include "poirot_msgs/msg/profiling_data.hpp"
#include "poirot_msgs/msg/system_info.hpp"
#include "rclcpp/rclcpp.hpp"

namespace poirot {

// ============================================================================
// Thread-Local Profiling Context
// ============================================================================
/**
 * @brief Context structure to hold per-thread profiling data.
 * This structure maintains the starting metrics for a thread when
 * profiling begins, allowing accurate measurement of resource usage
 * during the profiling session.
 */
struct ThreadProfilingContext {
  std::string function_name;
  std::chrono::high_resolution_clock::time_point start_time;
  double start_cpu_time_us = 0.0;
  double start_process_cpu_time_us = 0.0; // For energy attribution
  double start_system_cpu_time_us =
      0.0; // System-wide CPU time for accurate energy attribution
  long start_memory_kb = 0;
  long start_io_read_bytes = 0;
  long start_io_write_bytes = 0;
  long start_ctx_switches = 0;
  double start_energy_uj = 0.0;
  std::thread::id thread_id;
};

// ============================================================================
// Main Profiler Class
// ============================================================================
/**
 * @class Poirot
 * @brief Main class for the Poirot profiler.
 * This class provides methods to start and stop profiling, retrieve statistics,
 * and manage system and process information. It is designed to be thread-safe
 * and efficient for use in multi-threaded applications.
 */
class Poirot {
public:
  /**
   * @brief Constructor for the Poirot profiler.
   * Initializes system detection and ROS 2 publisher.
   */
  Poirot();

  /**
   * @brief Destructor for the Poirot profiler.
   */
  ~Poirot();

  /**
   * @brief Start profiling a function.
   * @param function_name Name of the function being profiled.
   */
  void start_profiling(const std::string &function_name);

  /**
   * @brief Stop profiling the current function.
   */
  void stop_profiling();

  /**
   * @brief Retrieve collected statistics.
   * @return Map of function names to their profiling statistics.
   */
  std::map<std::string, poirot_msgs::msg::FunctionStats> get_statistics() const;

  /**
   * @brief Get system information.
   * @return System information message.
   */
  poirot_msgs::msg::SystemInfo get_system_info() const;

  /**
   * @brief Get process information.
   * @return Process information message.
   */
  poirot_msgs::msg::ProcessInfo get_process_info() const;

  /**
   * @brief Print a summary of the collected statistics to the console.
   */
  void print_summary() const;

  /**
   * @brief Get the singleton instance of the Poirot profiler.
   * @return Reference to the singleton Poirot instance.
   */
  static Poirot &get_instance();

  /**
   * @brief Set verbosity level for logging.
   * @param verbose True to enable verbose logging, false to disable.
   */
  static void set_verbose(bool verbose);

  /**
   * @brief Print system information to the console.
   */
  static void print_system_info();

private:
  /**
   * @brief Auto-configure the profiler by detecting system info and loading CO2
   * factors.
   */
  void auto_configure();

  /**
   * @brief Detect system information such as CPU, RAM, and OS details.
   */
  void detect_system_info();

  /**
   * @brief Get country code from system timezone.
   * @return Country code as a string.
   */
  std::string get_country_from_timezone();

  /**
   * @brief Get CO2 factor for a given country.
   * @param country Country code.
   * @return CO2 factor in gCO2/kWh.
   */
  double get_co2_factor_for_country(const std::string &country);

  /**
   * @brief Search for hwmon paths for energy and power readings.
   */
  void search_hwmon_paths();

  /**
   * @brief Download CO2 factors from an external source.
   * @return True if download was successful, false otherwise.
   */
  bool download_co2_factors();

  /**
   * @brief Load CO2 factors from a local file.
   * @return True if loading was successful, false otherwise.
   */
  void set_co2_factor(double factor);

  /**
   * @brief Read current thread's CPU time in microseconds.
   * @return CPU time in microseconds.
   */
  double read_thread_cpu_time_us();

  /**
   * @brief Read current process's CPU time in microseconds.
   * @return Process CPU time in microseconds.
   */
  double read_process_cpu_time_us();

  /**
   * @brief Read system-wide CPU time in microseconds.
   * @return System-wide CPU time in microseconds.
   */
  double read_system_cpu_time_us();

  /**
   * @brief Read current thread's memory usage in kilobytes.
   * @return Memory usage in kilobytes.
   */
  long read_thread_memory_kb();

  /**
   * @brief Read current thread's I/O bytes.
   * @param read_bytes Reference to store read bytes.
   * @param write_bytes Reference to store write bytes.
   */
  void read_thread_io_bytes(long &read_bytes, long &write_bytes);

  /**
   * @brief Read current thread's context switches.
   * @return Number of context switches.
   */
  long read_thread_ctx_switches();

  /**
   * @brief Read energy consumption in microjoules.
   * @return Energy consumption in microjoules.
   */
  double read_energy_uj();

  /**
   * @brief Get battery power in watts.
   * @return Battery power in watts.
   */
  double get_battery_power_w();

  /**
   * @brief Calculate thread-level energy using hierarchical CPU time
   * attribution.
   * @param thread_cpu_delta_us Thread CPU time delta in microseconds.
   * @param process_cpu_delta_us Process CPU time delta in microseconds.
   */
  double calculate_thread_energy_uj(double thread_cpu_delta_us,
                                    double process_cpu_delta_us,
                                    double system_cpu_delta_us,
                                    double total_energy_delta_uj);

  /**
   * @brief Read process-level data such as memory and I/O.
   */
  void read_process_data();

  /**
   * @brief Read process CPU usage percentage.
   * @return CPU usage percentage.
   */
  double read_process_cpu_percent();

  /**
   * @brief Read number of threads in the process.
   * @return Number of threads.
   */
  int read_process_thread_count();

  /**
   * @brief Publish profiling statistics for a specific function.
   * @param function_name Name of the function whose statistics to publish.
   */
  void publish_stats(const std::string &function_name);

  /**
   * @brief Get the thread-local profiling context.
   * @return Reference to the thread-local profiling context.
   */
  ThreadProfilingContext &get_thread_context();

  /// @brief System information
  poirot_msgs::msg::SystemInfo system_info_;
  /// @brief Process information
  poirot_msgs::msg::ProcessInfo process_info_;

  /// @brief Verbosity flag
  std::atomic<bool> verbose_{false};

  /// @brief Storage (protected by shared_mutex for better read concurrency)
  mutable std::shared_mutex statistics_mutex_;
  /// @brief Profiling statistics
  std::map<std::string, poirot_msgs::msg::FunctionStats> statistics_;

  /// @brief Thread-local contexts storage
  mutable std::mutex contexts_mutex_;
  /// @brief Map of thread IDs to their profiling contexts
  std::map<std::thread::id, ThreadProfilingContext> thread_contexts_;

  /// @brief CPU usage tracking (for process)
  std::atomic<unsigned long long> prev_process_cpu_{0};
  /// @brief CPU usage tracking (for system)
  std::chrono::high_resolution_clock::time_point prev_cpu_read_time_;
  /// @brief Mutex for CPU read time updates
  mutable std::mutex cpu_read_mutex_;

  /// @brief Mutex for energy readings
  mutable std::mutex energy_mutex_;
  /// @brief Energy tracking variables
  double accumulated_energy_uj_ = 0.0;
  /// @brief Last energy read time point
  std::chrono::high_resolution_clock::time_point last_energy_read_time_;
  /// @brief RAPL energy tracking
  double last_rapl_energy_uj_ = 0.0;
  /// @brief Maximum RAPL energy value before wrap-around
  double rapl_max_energy_uj_ = 0.0;

  /// @brief Cached paths for hwmon (found once, used many times)
  std::string cached_hwmon_energy_path_;
  /// @brief Cached hwmon power path
  std::string cached_hwmon_power_path_;
  /// @brief Flag indicating if hwmon paths have been searched
  bool hwmon_paths_searched_ = false;

  /// @brief CO2 factors (dynamically loaded)
  mutable std::shared_mutex co2_factors_mutex_;
  /// @brief Map of country codes to their CO2 factors
  std::map<std::string, double> co2_factors_by_country_;
  /// @brief Flag indicating if CO2 factors have been loaded
  std::atomic<bool> co2_factors_loaded_{false};

  /// @brief ROS 2 Publisher for profiling data
  rclcpp::Publisher<poirot_msgs::msg::ProfilingData>::SharedPtr
      profiling_data_publisher_;
  /// @brief ROS 2 Node for publishing
  rclcpp::Node::SharedPtr node_;
};

// ============================================================================
// RAII Wrapper and Macros
// ============================================================================
/**
 * @class ScopedPoirot
 * @brief RAII class to automatically profile a function's execution.
 * This class starts profiling upon construction and stops profiling
 * upon destruction, ensuring that profiling is correctly managed even
 * in the presence of exceptions or early returns.
 */
class ScopedPoirot {
public:
  /**
   * @brief Constructor that starts profiling.
   * @param profiler Reference to the Poirot profiler instance.
   * @param func Name of the function being profiled.
   * @param file Source file name.
   * @param line Line number in the source file.
   */
  ScopedPoirot(Poirot &profiler, const char *func, const char *file, int line);

  /**
   * @brief Destructor that stops profiling.
   */
  ~ScopedPoirot();

  /**
   * @brief Deleted copy and move constructors and assignment operators.
   */
  ScopedPoirot(const ScopedPoirot &) = delete;

  /**
   * @brief Deleted copy and move constructors and assignment operators.
   */
  ScopedPoirot &operator=(const ScopedPoirot &) = delete;

  /**
   * @brief Deleted copy and move constructors and assignment operators.
   */
  ScopedPoirot(ScopedPoirot &&) = delete;

  /**
   * @brief Deleted copy and move constructors and assignment operators.
   */
  ScopedPoirot &operator=(ScopedPoirot &&) = delete;

private:
  /// @brief Reference to the Poirot profiler instance.
  Poirot &profiler_;
  /// @brief Extract function name from pretty function signature.
  static std::string extract_function_name(const char *pretty_function);
};

// Convenience macros
#define PROFILE_FUNCTION()                                                     \
  poirot::ScopedPoirot _profiler_guard_(                                       \
      poirot::Poirot::get_instance(), __PRETTY_FUNCTION__, __FILE__, __LINE__)

} // namespace poirot

#endif // POIROT__POIROT_HPP_
