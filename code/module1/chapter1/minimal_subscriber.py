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
Minimal subscriber example for Chapter 1: Introduction to ROS 2.

This example demonstrates:
- Creating a ROS 2 node
- Creating a subscriber
- Subscribing to messages
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalSubscriber(Node):

    def __init__(self):
        # Initialize the parent class (Node)
        super().__init__('minimal_subscriber')

        # Create a subscription that will receive String messages from the 'topic' topic
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)

        # Make sure the subscription is properly created
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # Log the received message
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create an instance of the MinimalSubscriber class
    minimal_subscriber = MinimalSubscriber()

    # Use the spin function to keep the node running and processing callbacks
    try:
        rclpy.spin(minimal_subscriber)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly (optional - otherwise it will be done automatically when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()