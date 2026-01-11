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

#include <chrono>
#include <functional>

#include "poirot_tui/tui_node.hpp"

using namespace std::chrono_literals;
using namespace poirot_tui;

TuiNode::TuiNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("poirot_tui", options), running_(false) {

  // Declare and get parameters
  this->declare_parameter<std::string>("topic", "/poirot/data");
  this->topic_name_ = this->get_parameter("topic").as_string();

  // Create data manager and renderer
  this->data_manager_ = std::make_unique<DataManager>();
  this->renderer_ = std::make_unique<TuiRenderer>();

  // Create subscription
  this->subscription_ =
      this->create_subscription<poirot_msgs::msg::ProfilingData>(
          this->topic_name_, rclcpp::QoS(100).reliable(),
          std::bind(&TuiNode::profiling_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Poirot TUI initialized, subscribing to: %s",
              this->topic_name_.c_str());
}

TuiNode::~TuiNode() { this->stop(); }

void TuiNode::run() {
  if (!this->renderer_->initialize()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize TUI renderer");
    return;
  }

  this->running_.store(true);

  // Create a timer for rendering at ~30 FPS
  this->render_timer_ =
      this->create_wall_timer(33ms, std::bind(&TuiNode::render_callback, this));

  RCLCPP_INFO(this->get_logger(), "TUI running...");
}

void TuiNode::stop() {
  this->running_.store(false);

  if (this->render_timer_) {
    this->render_timer_->cancel();
    this->render_timer_.reset();
  }

  if (this->renderer_) {
    this->renderer_->shutdown();
  }
}

void TuiNode::profiling_callback(
    const poirot_msgs::msg::ProfilingData::SharedPtr msg) {
  if (this->running_.load() && this->data_manager_) {
    this->data_manager_->process_profiling_data(msg);
  }
}

void TuiNode::render_callback() {
  if (!this->running_.load() || !this->renderer_ || !this->data_manager_) {
    return;
  }

  // Handle input
  if (!this->renderer_->handle_input(*this->data_manager_)) {
    this->stop();
    rclcpp::shutdown();
    return;
  }

  // Render
  this->renderer_->render(*this->data_manager_);
}
