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
Simple Listener Node Example for Chapter 5: ROS 2 Packages and Launch Files.

This example demonstrates a basic ROS 2 Python node that can be used in a package
and launched with launch files.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimpleListener(Node):
    """
    A simple listener node that subscribes to a topic.
    """

    def __init__(self, name, topic_name):
        super().__init__(name)
        self.subscription = self.create_subscription(
            String,
            topic_name,
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info(f'Simple listener node initialized with name: {name} on topic: {topic_name}')

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)

    simple_listener = SimpleListener('simple_listener', 'chatter')

    try:
        rclpy.spin(simple_listener)
    except KeyboardInterrupt:
        pass
    finally:
        simple_listener.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()