#!/usr/bin/env python3
# Copyright 2026 Miguel Ángel González Santamarta
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import math
import time
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String

from poirot import Poirot, profile_function


class PublisherNode(Node):
    """Demo publisher node that publishes messages and simulates work."""

    def __init__(self) -> None:
        """Initialize the publisher node."""
        super().__init__("python_publisher_node")

        # Create a publisher
        qos = QoSProfile(depth=10)
        self.publisher = self.create_publisher(String, "demo_topic", qos)

        # Create a timer that fires every 1 second
        self.timer = self.create_timer(1.0, self.timer_callback)

        # Persistent memory that grows
        self.persistent_memory: List[int] = []

        self.get_logger().info("Python Publisher Node initialized")

    @profile_function
    def timer_callback(self) -> None:
        """Timer callback that simulates work and publishes messages."""
        message = String()
        message.data = "Timer function executed (Python)"
        self.get_logger().info("Timer: %s" % message.data)

        # Simulate CPU work
        v = [0.0] * 10000
        for i in range(len(v)):
            v[i] = math.sin(i * 0.01) * math.cos(i * 0.01)

        # Simulate memory usage - persistent allocation
        old_size = len(self.persistent_memory)
        self.persistent_memory.extend([42] * 100000)  # Grow by 100KB each time

        # Simulate some wall time
        time.sleep(0.01)

        # Publish to trigger subscription
        self.publisher.publish(message)


class SubscriberNode(Node):
    """Demo subscriber node that receives messages and simulates work."""

    def __init__(self) -> None:
        """Initialize the subscriber node."""
        super().__init__("python_subscriber_node")

        # Create a subscription
        qos = QoSProfile(depth=10)
        self.subscription = self.create_subscription(
            String, "demo_topic", self.subscription_callback, qos
        )

        # Persistent memory that grows
        self.persistent_memory: List[int] = []

        self.get_logger().info("Python Subscriber Node initialized")

    @profile_function
    def subscription_callback(self, msg: String) -> None:
        """Subscription callback that simulates work upon receiving messages."""
        self.get_logger().info("Received: %s" % msg.data)

        # Simulate CPU work
        v = [0.0] * 5000
        for i in range(len(v)):
            v[i] = math.sqrt(i * 1.0)

        # Simulate memory usage - persistent allocation
        old_size = len(self.persistent_memory)
        self.persistent_memory.extend([42] * 50000)  # Grow by 50KB each time

        # Simulate some wall time
        time.sleep(0.005)


def main(args=None) -> None:
    """Main function to run the Python demo."""
    rclpy.init(args=args)

    # Print system info
    Poirot.print_system_info()

    # Create nodes
    publisher_node = PublisherNode()
    subscriber_node = SubscriberNode()

    # Use standard executor
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(publisher_node)
    executor.add_node(subscriber_node)

    print("\n=========================================")
    print("POIROT Python Demo")
    print("Measuring: CPU, RAM, I/O, Energy, CO2")
    print("Context switches, Page faults tracked!")
    print("All parameters auto-detected!")
    print("\nFor TUI mode, run in separate terminal:")
    print("  ros2 run poirot_tui poirot_tui")
    print("\nPress Ctrl+C to stop and see summary.")
    print("=========================================\n")

    Poirot.set_verbose(True)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        publisher_node.destroy_node()
        subscriber_node.destroy_node()


if __name__ == "__main__":
    main()
