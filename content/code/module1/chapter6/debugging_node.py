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
Debugging Node Example for Chapter 6: ROS 2 Tools and Debugging.

This example demonstrates various debugging techniques in ROS 2 including:
- Proper logging usage
- Parameter handling
- Service and topic debugging
- Error handling and recovery
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from example_interfaces.srv import AddTwoInts
import time
import traceback


class DebuggingNode(Node):
    """
    A node that demonstrates various debugging techniques in ROS 2.
    """

    def __init__(self):
        super().__init__('debugging_node')

        # Setup parameters with debugging
        self.declare_parameter('debug_level', 1)
        self.declare_parameter('message_interval', 1.0)
        self.declare_parameter('error_simulation', False)

        self.debug_level = self.get_parameter('debug_level').value
        self.message_interval = self.get_parameter('message_interval').value
        self.error_simulation = self.get_parameter('error_simulation').value

        self.get_logger().info(f'Initialized with debug level: {self.debug_level}')
        self.get_logger().info(f'Message interval: {self.message_interval}s')
        self.get_logger().info(f'Error simulation: {self.error_simulation}')

        # Create publisher with QoS for debugging
        qos_profile = QoSProfile(depth=10)
        self.publisher = self.create_publisher(String, 'debug_topic', qos_profile)

        # Create subscriber for debugging
        self.subscription = self.create_subscription(
            String,
            'debug_input',
            self.listener_callback,
            qos_profile
        )

        # Create service server for debugging
        self.srv = self.create_service(
            AddTwoInts,
            'debug_add_two_ints',
            self.add_two_ints_callback
        )

        # Timer for periodic messages
        self.timer = self.create_timer(self.message_interval, self.timer_callback)
        self.i = 0

        self.get_logger().info('Debugging node initialized successfully')

    def timer_callback(self):
        """Callback for the timer to publish debug messages."""
        try:
            msg = String()
            msg.data = f'Debug message {self.i}: time={time.time()}'

            # Simulate error for debugging purposes
            if self.error_simulation and self.i % 10 == 5:
                raise Exception("Simulated error for debugging")

            self.publisher.publish(msg)
            self.get_logger().debug(f'Published: "{msg.data}"')
            self.i += 1

        except Exception as e:
            self.get_logger().error(f'Error in timer_callback: {str(e)}')
            self.get_logger().error(f'Traceback: {traceback.format_exc()}')

            # Continue operation despite error
            self.i += 1

    def listener_callback(self, msg):
        """Callback for the subscriber to demonstrate debugging."""
        self.get_logger().info(f'Subscribed: "{msg.data}"')

        # Log with different levels based on debug level
        if self.debug_level >= 2:
            self.get_logger().debug(f'Detailed info: message length = {len(msg.data)}')
        if self.debug_level >= 3:
            self.get_logger().debug(f'Raw message: {repr(msg.data)}')

        # Simulate processing time for debugging
        time.sleep(0.01)

    def add_two_ints_callback(self, request, response):
        """Service callback for debugging service calls."""
        self.get_logger().info(f'Received service request: {request.a} + {request.b}')

        # Simulate computation time for debugging
        time.sleep(0.1)

        response.sum = request.a + request.b

        self.get_logger().info(f'Service response: {response.sum}')
        return response


def main(args=None):
    rclpy.init(args=args)

    debugging_node = DebuggingNode()

    # Add signal handlers for debugging
    import signal
    import sys

    def signal_handler(sig, frame):
        debugging_node.get_logger().info('Received SIGINT, shutting down')
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    try:
        rclpy.spin(debugging_node)
    except KeyboardInterrupt:
        debugging_node.get_logger().info('Caught keyboard interrupt')
    except Exception as e:
        debugging_node.get_logger().error(f'Unhandled exception: {str(e)}')
        debugging_node.get_logger().error(f'Traceback: {traceback.format_exc()}')
    finally:
        debugging_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()