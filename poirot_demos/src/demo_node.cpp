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

#include <chrono>
#include <cmath>
#include <thread>
#include <vector>

#include "poirot/poirot.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace poirot;

class PublisherNode : public rclcpp::Node {
public:
  PublisherNode() : Node("publisher_node") {
    // Create a publisher
    rclcpp::PublisherOptions pub_options;
    pub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;
    this->publisher_ =
        create_publisher<std_msgs::msg::String>("demo_topic", 10, pub_options);

    // Create a timer that fires every 1 second and publishes
    this->timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
      // Profile this timer function
      PROFILE_FUNCTION();

      auto message = std::make_shared<std_msgs::msg::String>();
      message->data = "Timer function executed";
      RCLCPP_INFO(this->get_logger(), "Timer: %s", message->data.c_str());

      // Simulate CPU work
      std::vector<double> v(10000);
      for (size_t i = 0; i < v.size(); ++i) {
        v[i] = std::sin(i * 0.01) * std::cos(i * 0.01);
      }

      // Simulate memory usage - persistent allocation
      size_t old_size = this->persistent_memory_.size();
      this->persistent_memory_.resize(old_size +
                                      100000); // Grow by 100KB each time
      std::fill(this->persistent_memory_.begin() + old_size,
                this->persistent_memory_.end(), 42);

      // Simulate some wall time
      std::this_thread::sleep_for(std::chrono::milliseconds(10));

      // Publish to trigger subscription
      this->publisher_->publish(*message);
    });
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  std::vector<char> persistent_memory_; // Persistent memory that grows
};

class SubscriberNode : public rclcpp::Node {
public:
  SubscriberNode() : Node("subscriber_node") {
    // Create a subscription
    rclcpp::SubscriptionOptions sub_options;
    sub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;
    this->subscription_ = create_subscription<std_msgs::msg::String>(
        "demo_topic", 10,
        std::bind(&SubscriberNode::subscription_function, this,
                  std::placeholders::_1),
        sub_options);
  }

private:
  void subscription_function(const std_msgs::msg::String::SharedPtr msg) {
    // Profile this subscription function
    PROFILE_FUNCTION();

    RCLCPP_INFO(get_logger(), "Received: %s", msg->data.c_str());

    // Simulate CPU work
    std::vector<double> v(5000);
    for (size_t i = 0; i < v.size(); ++i) {
      v[i] = std::sqrt(i * 1.0);
    }

    // Simulate memory usage - persistent allocation
    size_t old_size = this->persistent_memory_.size();
    this->persistent_memory_.resize(old_size + 50000); // Grow by 50KB each time
    std::fill(this->persistent_memory_.begin() + old_size,
              this->persistent_memory_.end(), 42);

    // Simulate some wall time
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  std::vector<char> persistent_memory_; // Persistent memory that grows
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  // Print system info
  Poirot::print_system_info();

  auto publisher_node = std::make_shared<PublisherNode>();
  auto subscriber_node = std::make_shared<SubscriberNode>();

  // Use standard executor - profiling is done via PROFILE_FUNCTION() macro
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(publisher_node);
  executor.add_node(subscriber_node);

  std::cout << "\n========================================\n";
  std::cout << "POIROT Demo\n";
  std::cout << "Measuring: CPU, RAM, I/O, Energy, CO2\n";
  std::cout << "Context switches, Page faults tracked!\n";
  std::cout << "All parameters auto-detected!\n";
  std::cout << "\nFor TUI mode, run in separate terminal:\n";
  std::cout << "  ros2 run poirot_tui poirot_tui\n";
  std::cout << "\nPress Ctrl+C to stop and see summary.\n";
  std::cout << "========================================\n\n";

  Poirot::set_verbose(true);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}