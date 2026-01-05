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

#ifndef POIROT_TUI__TUI_NODE_HPP_
#define POIROT_TUI__TUI_NODE_HPP_

#include <atomic>
#include <memory>
#include <string>

#include "poirot_msgs/msg/profiling_data.hpp"
#include "poirot_tui/data_manager.hpp"
#include "poirot_tui/tui_renderer.hpp"
#include "rclcpp/rclcpp.hpp"

namespace poirot_tui {

/// ROS 2 Node for the TUI
class TuiNode : public rclcpp::Node {
public:
  explicit TuiNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~TuiNode() override;

  /// Run the TUI main loop
  void run();

  /// Stop the TUI
  void stop();

  /// Check if running
  bool isRunning() const { return this->running_.load(); }

private:
  /// Callback for profiling data
  void profilingCallback(const poirot_msgs::msg::ProfilingData::SharedPtr msg);

  /// Timer callback for rendering
  void renderCallback();

  rclcpp::Subscription<poirot_msgs::msg::ProfilingData>::SharedPtr
      subscription_;
  rclcpp::TimerBase::SharedPtr render_timer_;

  std::unique_ptr<DataManager> data_manager_;
  std::unique_ptr<TuiRenderer> renderer_;

  std::atomic<bool> running_;
  std::string topic_name_;
};

} // namespace poirot_tui

#endif // POIROT_TUI__TUI_NODE_HPP_
