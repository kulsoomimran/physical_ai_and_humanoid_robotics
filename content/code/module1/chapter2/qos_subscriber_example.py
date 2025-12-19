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
QoS Subscriber Example for Chapter 2: ROS 2 Architecture and Communication.

This example demonstrates:
- Subscribing with different QoS profiles
- How QoS policies affect message reception
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String


class QoSSubscriberExample(Node):

    def __init__(self):
        super().__init__('qos_subscriber_example')

        # Subscribe to reliable topic with matching QoS
        reliable_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )
        self.reliable_subscriber = self.create_subscription(
            String,
            'reliable_topic',
            self.reliable_callback,
            reliable_qos
        )

        # Subscribe to best effort topic
        best_effort_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )
        self.best_effort_subscriber = self.create_subscription(
            String,
            'best_effort_topic',
            self.best_effort_callback,
            best_effort_qos
        )

        # Subscribe to config topic with transient local durability
        config_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST
        )
        self.config_subscriber = self.create_subscription(
            String,
            'config_topic',
            self.config_callback,
            config_qos
        )

        # Subscribe to log topic
        log_qos = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_ALL
        )
        self.log_subscriber = self.create_subscription(
            String,
            'log_topic',
            self.log_callback,
            log_qos
        )

    def reliable_callback(self, msg):
        self.get_logger().info(f'Received from reliable topic: "{msg.data}"')

    def best_effort_callback(self, msg):
        self.get_logger().info(f'Received from best effort topic: "{msg.data}"')

    def config_callback(self, msg):
        self.get_logger().info(f'Received from config topic: "{msg.data}"')

    def log_callback(self, msg):
        self.get_logger().info(f'Received from log topic: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)

    qos_subscriber_example = QoSSubscriberExample()

    try:
        rclpy.spin(qos_subscriber_example)
    except KeyboardInterrupt:
        pass

    qos_subscriber_example.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()