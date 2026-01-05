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

class PoirotRecorderNode : public rclcpp::Node {
public:
  PoirotRecorderNode();
  ~PoirotRecorderNode();

private:
  void write_csv_header();
  void data_callback(const poirot_msgs::msg::ProfilingData::SharedPtr msg);
  std::string escape_csv(const std::string &field);

  rclcpp::Subscription<poirot_msgs::msg::ProfilingData>::SharedPtr
      subscription_;
  std::ofstream csv_file_;
  std::string csv_file_path_;
  std::mutex csv_mutex_;
  size_t record_count_;
};

#endif // POIROT_RECORDER__POIROT_RECORDER_NODE_HPP_
