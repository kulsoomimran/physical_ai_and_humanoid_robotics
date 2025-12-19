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
Simple Service Example for Chapter 4: ROS 2 Services and Actions.

This example demonstrates:
- Creating a ROS 2 service server
- Creating a ROS 2 service client
- Request-response communication pattern
"""

import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class SimpleServiceServer(Node):
    """
    A service server that adds two integers.
    """
    def __init__(self):
        super().__init__('simple_service_server')

        # Create a service that will use the callback function to process requests
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback)

        self.get_logger().info('Service server initialized')

    def add_two_ints_callback(self, request, response):
        # Perform the addition
        response.sum = request.a + request.b

        # Log the request and response
        self.get_logger().info(f'Incoming request: {request.a} + {request.b} = {response.sum}')

        return response


class SimpleServiceClient(Node):
    """
    A service client that calls the add_two_ints service.
    """
    def __init__(self):
        super().__init__('simple_service_client')

        # Create a client for the AddTwoInts service
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for the service to become available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        # Set the request parameters
        self.req.a = a
        self.req.b = b

        # Call the service asynchronously
        self.future = self.cli.call_async(self.req)
        self.get_logger().info(f'Sending request: {a} + {b}')


def main(args=None):
    rclpy.init(args=args)

    # Create service server node
    service_server = SimpleServiceServer()

    # Create service client node
    service_client = SimpleServiceClient()

    # Send a request from the client
    service_client.send_request(4, 5)

    # Use MultiThreadedExecutor to run both nodes simultaneously
    try:
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(service_server)
        executor.add_node(service_client)

        # Continue spinning until the response is received
        while rclpy.ok():
            executor.spin_once(timeout_sec=1.0)

            # Check if the client received a response
            if service_client.future.done():
                try:
                    response = service_client.future.result()
                    service_client.get_logger().info(f'Result of {service_client.req.a} + {service_client.req.b} = {response.sum}')
                except Exception as e:
                    service_client.get_logger().error(f'Service call failed: {e}')
                break

    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        service_server.destroy_node()
        service_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()