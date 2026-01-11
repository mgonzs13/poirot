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

/**
 * @class TuiNode
 * @brief TUI Node for displaying profiling data
 */
class TuiNode : public rclcpp::Node {
public:
  /**
   * @brief Constructor for TuiNode
   * @param options Node options
   */
  explicit TuiNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  /**
   * @brief Destructor for TuiNode
   */
  ~TuiNode() override;

  /**
   * @brief Run the TUI
   */
  void run();

  /**
   * @brief Stop the TUI
   */
  void stop();

  /**
   * @brief Check if running
   * @return True if running, false otherwise
   */
  bool is_running() const { return this->running_.load(); }

private:
  /**
   * @brief Callback for profiling data subscription
   * @param msg Shared pointer to the profiling data message
   */
  void profiling_callback(const poirot_msgs::msg::ProfilingData::SharedPtr msg);

  /**
   * @brief Timer callback for rendering
   */
  void render_callback();

  /// @brief Subscription to profiling data
  rclcpp::Subscription<poirot_msgs::msg::ProfilingData>::SharedPtr
      subscription_;
  /// @brief Timer for periodic rendering
  rclcpp::TimerBase::SharedPtr render_timer_;

  /// @brief Data manager for profiling data
  std::unique_ptr<DataManager> data_manager_;
  /// @brief TUI renderer
  std::unique_ptr<TuiRenderer> renderer_;

  /// @brief Running state
  std::atomic<bool> running_;
  /// @brief Topic name for profiling data
  std::string topic_name_;
};

} // namespace poirot_tui

#endif // POIROT_TUI__TUI_NODE_HPP_
