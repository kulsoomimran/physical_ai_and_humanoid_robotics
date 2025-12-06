#!/usr/bin/env python3
# Copyright 2025 Open Source Robotics Foundation, Inc.
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

"""
QoS Examples for Chapter 2: ROS 2 Architecture and Communication.

This example demonstrates:
- Different QoS policies in ROS 2
- How to configure reliability, durability, and history policies
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String


class QoSPublisherExample(Node):

    def __init__(self):
        super().__init__('qos_publisher_example')

        # Example 1: Reliable QoS profile for critical data
        reliable_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )
        self.reliable_publisher = self.create_publisher(String, 'reliable_topic', reliable_qos)

        # Example 2: Best effort QoS profile for high-frequency data
        best_effort_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )
        self.best_effort_publisher = self.create_publisher(String, 'best_effort_topic', best_effort_qos)

        # Example 3: Transient local durability for configuration data
        config_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST
        )
        self.config_publisher = self.create_publisher(String, 'config_topic', config_qos)

        # Example 4: Keep all history for logging
        log_qos = QoSProfile(
            depth=100,  # Large depth to keep more messages
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_ALL
        )
        self.log_publisher = self.create_publisher(String, 'log_topic', log_qos)

        # Create a timer to publish messages
        self.i = 0
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        # Create message
        msg = String()

        # Publish to reliable topic
        msg.data = f'Reliable message {self.i}'
        self.reliable_publisher.publish(msg)
        self.get_logger().info(f'Published to reliable topic: "{msg.data}"')

        # Publish to best effort topic
        msg.data = f'Best effort message {self.i}'
        self.best_effort_publisher.publish(msg)
        self.get_logger().info(f'Published to best effort topic: "{msg.data}"')

        # Publish to config topic (only publish once as example)
        if self.i == 0:
            msg.data = f'Configuration data - initial value'
            self.config_publisher.publish(msg)
            self.get_logger().info(f'Published to config topic: "{msg.data}"')

        # Publish to log topic
        msg.data = f'Log message {self.i}'
        self.log_publisher.publish(msg)
        self.get_logger().info(f'Published to log topic: "{msg.data}"')

        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    qos_publisher_example = QoSPublisherExample()

    try:
        rclpy.spin(qos_publisher_example)
    except KeyboardInterrupt:
        pass

    qos_publisher_example.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()