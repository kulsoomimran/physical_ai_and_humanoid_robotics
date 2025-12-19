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
Node and Topic Example for Chapter 3: ROS 2 Nodes and Topics.

This example demonstrates:
- Creating a ROS 2 node
- Creating publishers and subscribers
- Topic-based communication
- Proper node lifecycle management
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String


class NodeTopicPublisher(Node):
    """
    A publisher node that demonstrates creating nodes and publishers.
    """
    def __init__(self):
        # Initialize the parent class (Node) with the node name
        super().__init__('node_topic_publisher')

        # Create a QoS profile for the publisher
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )

        # Create a publisher that will publish String messages to the 'chatter' topic
        self.publisher_ = self.create_publisher(String, 'chatter', qos_profile)

        # Create a timer that calls the timer_callback method every 0.5 seconds (2 Hz)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Initialize a counter for the messages
        self.i = 0

        # Log that the publisher node has started
        self.get_logger().info('Node Topic Publisher node started')

    def timer_callback(self):
        # Create a String message
        msg = String()
        msg.data = f'Hello, ROS 2! Message #{self.i}'

        # Publish the message
        self.publisher_.publish(msg)

        # Log the message
        self.get_logger().info(f'Publishing: "{msg.data}"')

        # Increment the counter
        self.i += 1


class NodeTopicSubscriber(Node):
    """
    A subscriber node that demonstrates creating nodes and subscribers.
    """
    def __init__(self):
        # Initialize the parent class (Node) with the node name
        super().__init__('node_topic_subscriber')

        # Create a QoS profile for the subscriber (should match publisher)
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )

        # Create a subscription that will receive String messages from the 'chatter' topic
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            qos_profile)

        # Make sure the subscription is properly created
        self.subscription  # prevent unused variable warning

        # Log that the subscriber node has started
        self.get_logger().info('Node Topic Subscriber node started')

    def listener_callback(self, msg):
        # Log the received message
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create instances of both publisher and subscriber nodes
    publisher_node = NodeTopicPublisher()
    subscriber_node = NodeTopicSubscriber()

    # Use MultiThreadedExecutor to run both nodes simultaneously
    try:
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(publisher_node)
        executor.add_node(subscriber_node)

        # Spin both nodes
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up: destroy nodes explicitly
        publisher_node.destroy_node()
        subscriber_node.destroy_node()

        # Shutdown the rclpy library
        rclpy.shutdown()


if __name__ == '__main__':
    main()