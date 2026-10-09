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
  FILE *pipe = popen(
      "sh -c 'i=0; while [ $i -lt 2000000 ]; do i=$((i+1)); done'", "r");
  ASSERT_NE(pipe, nullptr);
  char buffer[128];
  while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
  }
  pclose(pipe);
}

class ChildProfilingTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    int argc = 0;
    rclcpp::init(argc, nullptr);
  }

  static void TearDownTestSuite() { rclcpp::shutdown(); }
};

} // namespace

void profiled_busy_child() {
  PROFILE_FUNCTION_WITH_CHILDREN();
  burn_cpu_child();
}

TEST_F(ChildProfilingTest, FoldsReapedChildIntoCpuTime) {
  auto node = rclcpp::Node::make_shared("child_profiling_fold_node");
  poirot_msgs::msg::ProfilingData::SharedPtr received;
  auto subscription =
      node->create_subscription<poirot_msgs::msg::ProfilingData>(
          "poirot/data", rclcpp::QoS(100),
          [&received](poirot_msgs::msg::ProfilingData::SharedPtr msg) {
            if (msg->function.name == "ChildProfilingTestWindow") {
              received = msg;
            }
          });

  auto &profiler = poirot::Poirot::get_instance();

  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (subscription->get_publisher_count() == 0 &&
         std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  profiler.start_profiling("ChildProfilingTestWindow", __FILE__, __LINE__, true);
  burn_cpu_child();
  profiler.stop_profiling();

  deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (!received && std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  ASSERT_NE(received, nullptr);
  EXPECT_GT(received->function.call.data.child_cpu_time_us, 0);
  EXPECT_GE(received->function.call.data.cpu_time_us,
            received->function.call.data.child_cpu_time_us);
  EXPECT_LE(received->function.call.data.child_energy_uj,
            received->function.call.data.cpu_energy_uj);
}

TEST_F(ChildProfilingTest, FlagOffReportsNoChildUsage) {
  auto node = rclcpp::Node::make_shared("child_profiling_off_node");
  poirot_msgs::msg::ProfilingData::SharedPtr received;
  auto subscription =
      node->create_subscription<poirot_msgs::msg::ProfilingData>(
          "poirot/data", rclcpp::QoS(100),
          [&received](poirot_msgs::msg::ProfilingData::SharedPtr msg) {
            if (msg->function.name == "ChildProfilingOffWindow") {
              received = msg;
            }
          });

  auto &profiler = poirot::Poirot::get_instance();

  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (subscription->get_publisher_count() == 0 &&
         std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  profiler.start_profiling("ChildProfilingOffWindow", __FILE__, __LINE__);
  burn_cpu_child();
  profiler.stop_profiling();

  deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (!received && std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  ASSERT_NE(received, nullptr);
  EXPECT_EQ(received->function.call.data.child_cpu_time_us, 0);
  EXPECT_DOUBLE_EQ(received->function.call.data.child_energy_uj, 0.0);
}

TEST_F(ChildProfilingTest, MacroWithChildrenProfilesBusyChild) {
  auto node = rclcpp::Node::make_shared("child_profiling_macro_node");
  poirot_msgs::msg::ProfilingData::SharedPtr received;
  auto subscription =
      node->create_subscription<poirot_msgs::msg::ProfilingData>(
          "poirot/data", rclcpp::QoS(100),
          [&received](poirot_msgs::msg::ProfilingData::SharedPtr msg) {
            if (msg->function.name == "profiled_busy_child") {
              received = msg;
            }
          });

  poirot::Poirot::get_instance();

  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (subscription->get_publisher_count() == 0 &&
         std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  profiled_busy_child();

  deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (!received && std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  ASSERT_NE(received, nullptr);
  EXPECT_GT(received->function.call.data.child_cpu_time_us, 0);
}
