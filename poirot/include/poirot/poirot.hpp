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

#ifndef POIROT__FUNCTION_PROFILER_HPP_
#define POIROT__FUNCTION_PROFILER_HPP_

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
class Poirot {
public:
  Poirot();
  ~Poirot();

  // Profiling control (thread-safe)
  void start_profiling(const std::string &function_name);
  void stop_profiling();

  // Getters (return copies for thread safety)
  std::map<std::string, poirot_msgs::msg::FunctionStats> get_statistics() const;
  poirot_msgs::msg::SystemInfo get_system_info() const;
  poirot_msgs::msg::ProcessInfo get_process_info() const;

  // Output
  void print_summary() const;

  // Static accessor for internal use
  static Poirot &get_instance();

  // Static functions for external use
  static void set_verbose(bool verbose);
  static void enable_publishing(rclcpp::Node::SharedPtr node);
  static void disable_publishing();
  static void print_system_info();

private:
  // System detection
  void auto_configure();
  void detect_system_info();
  std::string get_country_from_timezone();
  double get_co2_factor_for_country(const std::string &country);
  void search_hwmon_paths();
  bool download_co2_factors();
  void set_co2_factor(double factor);

  // Thread-aware measurement functions
  double read_thread_cpu_time_us();
  double
  read_process_cpu_time_us(); // Total process CPU time for energy attribution
  double
  read_system_cpu_time_us(); // System-wide CPU time (all cores, all processes)
  long read_thread_memory_kb();
  void read_thread_io_bytes(long &read_bytes, long &write_bytes);
  long read_thread_ctx_switches();
  double read_energy_uj();
  double get_battery_power_w();
  double calculate_thread_energy_uj(double thread_cpu_delta_us,
                                    double process_cpu_delta_us,
                                    double system_cpu_delta_us,
                                    double total_energy_delta_uj);

  // Process measurements
  void read_process_data();
  double read_process_cpu_percent();
  int read_process_thread_count();

  // Publishing
  void publish_stats(const std::string &function_name);

  // Get current thread's profiling context
  ThreadProfilingContext &get_thread_context();

  // System information
  poirot_msgs::msg::SystemInfo system_info_;

  // Process information
  poirot_msgs::msg::ProcessInfo process_info_;

  // Configuration
  std::atomic<bool> verbose_{false};

  // Storage (protected by shared_mutex for better read concurrency)
  mutable std::shared_mutex statistics_mutex_;
  std::map<std::string, poirot_msgs::msg::FunctionStats> statistics_;

  // Thread-local contexts storage
  mutable std::mutex contexts_mutex_;
  std::map<std::thread::id, ThreadProfilingContext> thread_contexts_;

  // CPU usage tracking (for process)
  std::atomic<unsigned long long> prev_process_cpu_{0};
  std::chrono::high_resolution_clock::time_point prev_cpu_read_time_;
  mutable std::mutex cpu_read_mutex_;

  // Energy tracking (protected for thread safety)
  mutable std::mutex energy_mutex_;
  double accumulated_energy_uj_ = 0.0;
  std::chrono::high_resolution_clock::time_point last_energy_read_time_;
  double last_rapl_energy_uj_ = 0.0;
  double rapl_max_energy_uj_ = 0.0;

  // Cached paths for hwmon (found once, used many times)
  std::string cached_hwmon_energy_path_;
  std::string cached_hwmon_power_path_;
  bool hwmon_paths_searched_ = false;

  // CO2 factors (dynamically loaded)
  mutable std::shared_mutex co2_factors_mutex_;
  std::map<std::string, double> co2_factors_by_country_;
  std::atomic<bool> co2_factors_loaded_{false};

  // ROS 2 publishing
  rclcpp::Publisher<poirot_msgs::msg::ProfilingData>::SharedPtr
      profiling_data_publisher_;
  rclcpp::Node::SharedPtr node_;
  std::atomic<bool> publishing_enabled_{false};
};

// ============================================================================
// RAII Wrapper and Macros
// ============================================================================
class ScopedPoirot {
public:
  ScopedPoirot(Poirot &profiler, const char *func, const char *file, int line);
  ~ScopedPoirot();

  // Non-copyable, non-movable
  ScopedPoirot(const ScopedPoirot &) = delete;
  ScopedPoirot &operator=(const ScopedPoirot &) = delete;
  ScopedPoirot(ScopedPoirot &&) = delete;
  ScopedPoirot &operator=(ScopedPoirot &&) = delete;

private:
  Poirot &profiler_;
  static std::string extract_function_name(const char *pretty_function);
};

// Convenience macros
#define PROFILE_FUNCTION()                                                     \
  poirot::ScopedPoirot _profiler_guard_(                                       \
      poirot::Poirot::get_instance(), __PRETTY_FUNCTION__, __FILE__, __LINE__)

} // namespace poirot

#endif // POIROT__FUNCTION_PROFILER_HPP_
