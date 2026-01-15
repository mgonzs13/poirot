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
#include <map>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "poirot/utils/co2_manager.hpp"
#include "poirot/utils/energy_monitor.hpp"
#include "poirot/utils/gpu_monitor.hpp"
#include "poirot/utils/hwmon_scanner.hpp"
#include "poirot/utils/power_estimator.hpp"
#include "poirot/utils/process_metrics.hpp"
#include "poirot/utils/string_utils.hpp"
#include "poirot/utils/sysfs_reader.hpp"
#include "poirot/utils/thread_metrics.hpp"

#include "poirot_msgs/msg/function_stats.hpp"
#include "poirot_msgs/msg/process_info.hpp"
#include "poirot_msgs/msg/profiling_data.hpp"
#include "poirot_msgs/msg/system_info.hpp"

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
  std::chrono::steady_clock::time_point start_time;
  int64_t start_cpu_time_us = 0;
  int64_t start_process_cpu_time_us = 0;
  double start_gpu_utilization_percent = 0.0;
  int64_t start_mem_kb = 0;
  int64_t start_gpu_mem_kb = 0;
  int64_t start_io_read_bytes = 0;
  int64_t start_io_write_bytes = 0;
  int64_t start_context_switches = 0;
  double start_cpu_energy_uj = 0.0;
  double start_gpu_energy_uj = 0.0;
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
  ~Poirot() = default;

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
   * @brief Read process-level data such as memory and I/O.
   */
  void read_process_data();

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

  // ============================================================================
  // Utility Class Instances
  // ============================================================================
  /// @brief CO2 factor manager
  utils::Co2Manager co2_manager_;
  /// @brief Hardware monitoring scanner
  utils::HwmonScanner hwmon_scanner_;
  /// @brief Power estimator (depends on hwmon_scanner_)
  utils::PowerEstimator power_estimator_;
  /// @brief Energy monitor (depends on hwmon_scanner_)
  utils::EnergyMonitor energy_monitor_;
  /// @brief GPU monitor for GPU metrics and energy
  utils::GpuMonitor gpu_monitor_;
  /// @brief Process metrics tracker
  utils::ProcessMetrics process_metrics_;
  /// @brief Thread metrics tracker
  utils::ThreadMetrics thread_metrics_;

  // ============================================================================
  // ROS 2 Integration
  // ============================================================================
  /// @brief ROS 2 Node for publishing
  rclcpp::Node::SharedPtr poirot_node_;
  /// @brief ROS 2 Publisher for profiling data
  rclcpp::Publisher<poirot_msgs::msg::ProfilingData>::SharedPtr
      profiling_data_publisher_;
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
