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

#ifndef POIROT_RECORDER__POIROT_RECORDER_NODE_HPP_
#define POIROT_RECORDER__POIROT_RECORDER_NODE_HPP_

#include <fstream>
#include <mutex>
#include <string>

#include "poirot_msgs/msg/profiling_data.hpp"
#include "rclcpp/rclcpp.hpp"

/**
 * @brief Node that records profiling data to a CSV file.
 */
class PoirotRecorderNode : public rclcpp::Node {
public:
  /**
   * @brief Constructor for PoirotRecorderNode.
   */
  PoirotRecorderNode();

  /**
   * @brief Destructor for PoirotRecorderNode.
   */
  ~PoirotRecorderNode();

private:
  /**
   * @brief Write the CSV header to the file.
   */
  void write_csv_header();

  /**
   * @brief Callback function to handle incoming profiling data messages.
   */
  void data_callback(const poirot_msgs::msg::ProfilingData::SharedPtr msg);

  /**
   * @brief Escape a field for CSV format.
   * @param field The field to escape.
   * @return The escaped field.
   */
  std::string escape_csv(const std::string &field);

  /// @brief Subscription to profiling data messages.
  rclcpp::Subscription<poirot_msgs::msg::ProfilingData>::SharedPtr
      subscription_;
  /// @brief Output CSV file stream.
  std::ofstream csv_file_;
  /// @brief Path to the CSV file.
  std::string csv_file_path_;
  /// @brief Mutex for thread-safe CSV writing.
  std::mutex csv_mutex_;
  /// @brief Count of recorded entries.
  size_t record_count_;
};

#endif // POIROT_RECORDER__POIROT_RECORDER_NODE_HPP_
