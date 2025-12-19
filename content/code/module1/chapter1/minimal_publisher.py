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
Minimal publisher example for Chapter 1: Introduction to ROS 2.

This example demonstrates:
- Creating a ROS 2 node
- Creating a publisher
- Publishing messages
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        # Initialize the parent class (Node)
        super().__init__('minimal_publisher')

        # Create a publisher that will publish String messages to the 'topic' topic
        self.publisher_ = self.create_publisher(String, 'topic', 10)

        # Create a timer that calls the timer_callback method every 0.5 seconds (2 Hz)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Initialize a counter for the messages
        self.i = 0

    def timer_callback(self):
        # Create a String message
        msg = String()
        msg.data = f'Hello World: {self.i}'

        # Publish the message
        self.publisher_.publish(msg)

        # Log the message
        self.get_logger().info(f'Publishing: "{msg.data}"')

        # Increment the counter
        self.i += 1


def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create an instance of the MinimalPublisher class
    minimal_publisher = MinimalPublisher()

    # Use the spin function to keep the node running and processing callbacks
    try:
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly (optional - otherwise it will be done automatically when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()