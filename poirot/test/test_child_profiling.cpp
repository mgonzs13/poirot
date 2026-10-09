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

#include <gtest/gtest.h>
#include <sys/resource.h>

#include <chrono>
#include <cstdio>
#include <map>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "poirot/poirot.hpp"
#include "poirot_msgs/msg/profiling_data.hpp"

TEST(RusageCpuDeltaTest, AddsUserAndSystemTime) {
  struct rusage before {};
  struct rusage after {};
  before.ru_utime = {1, 500000};
  before.ru_stime = {0, 250000};
  after.ru_utime = {2, 100000};
  after.ru_stime = {0, 300000};
  // (2.1 - 1.5) + (0.30 - 0.25) = 0.65 s = 650000 us.
  EXPECT_DOUBLE_EQ(poirot::Poirot::rusage_cpu_delta_us(before, after),
                   650000.0);
}

TEST(RusageCpuDeltaTest, HandlesMicrosecondCarry) {
  struct rusage before {};
  struct rusage after {};
  before.ru_utime = {1, 900000};
  after.ru_utime = {2, 100000};
  EXPECT_DOUBLE_EQ(poirot::Poirot::rusage_cpu_delta_us(before, after),
                   200000.0);
}

TEST(RusageCpuDeltaTest, ReturnsZeroForEqualUsage) {
  struct rusage usage {};
  usage.ru_utime = {3, 0};
  usage.ru_stime = {1, 0};
  EXPECT_DOUBLE_EQ(poirot::Poirot::rusage_cpu_delta_us(usage, usage), 0.0);
}

TEST(RusageCpuDeltaTest, ClampsNegativeDeltaToZero) {
  struct rusage before {};
  struct rusage after {};
  before.ru_utime = {2, 0};
  before.ru_stime = {1, 0};
  after.ru_utime = {1, 0};
  after.ru_stime = {0, 500000};
  EXPECT_DOUBLE_EQ(poirot::Poirot::rusage_cpu_delta_us(before, after), 0.0);
}

namespace {

void burn_cpu_child() {
  FILE *pipe =
      popen("sh -c 'i=0; while [ $i -lt 2000000 ]; do i=$((i+1)); done'", "r");
  ASSERT_NE(pipe, nullptr);
  char buffer[128];
  while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
  }
  pclose(pipe);
}

class ChildProfilingTest : public ::testing::Test {
protected:
  static void SetUpTestCase() {
    int argc = 0;
    rclcpp::init(argc, nullptr);

    // A single node and subscription live for the whole suite. Creating and
    // destroying ROS 2 participants between tests is unreliable on Foxy's
    // FastRTPS (its discovery listener thread can shut down), which prevents
    // later tests from receiving published messages.
    node_ = rclcpp::Node::make_shared("child_profiling_test_node");
    subscription_ = node_->create_subscription<poirot_msgs::msg::ProfilingData>(
        "poirot/data", rclcpp::QoS(100),
        [](poirot_msgs::msg::ProfilingData::SharedPtr msg) {
          received_[msg->function.name] = msg;
        });
  }

  static void TearDownTestCase() {
    subscription_.reset();
    node_.reset();
    received_.clear();
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void TearDown() override { received_.clear(); }

  static void wait_for_publisher() {
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (subscription_->get_publisher_count() == 0 &&
           std::chrono::steady_clock::now() < deadline) {
      rclcpp::spin_some(node_);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  static bool
  wait_for_message(const std::string &function_name,
                   poirot_msgs::msg::ProfilingData::SharedPtr &received) {
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (std::chrono::steady_clock::now() < deadline) {
      rclcpp::spin_some(node_);
      auto it = received_.find(function_name);
      if (it != received_.end()) {
        received = it->second;
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
  }

  static rclcpp::Node::SharedPtr node_;
  static rclcpp::Subscription<poirot_msgs::msg::ProfilingData>::SharedPtr
      subscription_;
  static std::map<std::string, poirot_msgs::msg::ProfilingData::SharedPtr>
      received_;
};

rclcpp::Node::SharedPtr ChildProfilingTest::node_;
rclcpp::Subscription<poirot_msgs::msg::ProfilingData>::SharedPtr
    ChildProfilingTest::subscription_;
std::map<std::string, poirot_msgs::msg::ProfilingData::SharedPtr>
    ChildProfilingTest::received_;

} // namespace

void profiled_busy_child() {
  PROFILE_FUNCTION_WITH_CHILDREN();
  burn_cpu_child();
}

TEST_F(ChildProfilingTest, FoldsReapedChildIntoCpuTime) {
  auto &profiler = poirot::Poirot::get_instance();
  wait_for_publisher();

  profiler.start_profiling("ChildProfilingTestWindow", __FILE__, __LINE__,
                           true);
  burn_cpu_child();
  profiler.stop_profiling();

  poirot_msgs::msg::ProfilingData::SharedPtr received;
  ASSERT_TRUE(wait_for_message("ChildProfilingTestWindow", received));
  EXPECT_GT(received->function.call.data.child_cpu_time_us, 0);
  EXPECT_GE(received->function.call.data.cpu_time_us,
            received->function.call.data.child_cpu_time_us);
  EXPECT_LE(received->function.call.data.child_energy_uj,
            received->function.call.data.cpu_energy_uj);
}

TEST_F(ChildProfilingTest, FlagOffReportsNoChildUsage) {
  auto &profiler = poirot::Poirot::get_instance();
  wait_for_publisher();

  profiler.start_profiling("ChildProfilingOffWindow", __FILE__, __LINE__);
  burn_cpu_child();
  profiler.stop_profiling();

  poirot_msgs::msg::ProfilingData::SharedPtr received;
  ASSERT_TRUE(wait_for_message("ChildProfilingOffWindow", received));
  EXPECT_EQ(received->function.call.data.child_cpu_time_us, 0);
  EXPECT_DOUBLE_EQ(received->function.call.data.child_energy_uj, 0.0);
}

TEST_F(ChildProfilingTest, MacroWithChildrenProfilesBusyChild) {
  poirot::Poirot::get_instance();
  wait_for_publisher();

  profiled_busy_child();

  poirot_msgs::msg::ProfilingData::SharedPtr received;
  ASSERT_TRUE(wait_for_message("profiled_busy_child", received));
  EXPECT_GT(received->function.call.data.child_cpu_time_us, 0);
}
