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
Simple Action Example for Chapter 4: ROS 2 Services and Actions.

This example demonstrates:
- Creating a ROS 2 action server
- Creating a ROS 2 action client
- Long-running operations with feedback and results
"""

import time
import rclpy
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from example_interfaces.action import Fibonacci


class SimpleActionServer(Node):
    """
    An action server that computes Fibonacci sequences with feedback.
    """
    def __init__(self):
        super().__init__('simple_action_server')

        # Create an action server
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)

        self.get_logger().info('Action server initialized')

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """Execute the goal."""
        self.get_logger().info('Executing goal...')

        # Feedback and result
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        result = Fibonacci.Result()

        # Calculate Fibonacci sequence up to the requested order
        for i in range(1, goal_handle.request.order):
            # Check if there was a cancel request
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                result.sequence = feedback_msg.sequence
                return result

            # Update the sequence
            feedback_msg.sequence.append(feedback_msg.sequence[i] + feedback_msg.sequence[i-1])

            # Publish feedback
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Feedback: {feedback_msg.sequence}')

            # Simulate some work
            time.sleep(1.0)

        # Complete the goal
        goal_handle.succeed()
        result.sequence = feedback_msg.sequence

        self.get_logger().info(f'Result: {result.sequence}')
        return result


class SimpleActionClient(Node):
    """
    An action client that sends goals to the Fibonacci action server.
    """
    def __init__(self):
        super().__init__('simple_action_client')

        # Create an action client
        self._action_client = ActionClient(
            self,
            Fibonacci,
            'fibonacci')

    def send_goal(self, order):
        """Send a goal to the action server."""
        # Wait for the action server to be available
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        # Create a goal
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        # Send the goal
        self.get_logger().info(f'Sending goal: order={order}')
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        # Set up a callback to handle the result
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle the goal response."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        # Get the result
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Handle the result."""
        result = future.result().result
        self.get_logger().info(f'Result: {result.sequence}')

    def feedback_callback(self, feedback_msg):
        """Handle feedback from the action server."""
        self.get_logger().info(f'Received feedback: {feedback_msg.feedback.sequence}')


def main(args=None):
    rclpy.init(args=args)

    # Create action server and client nodes
    action_server = SimpleActionServer()
    action_client = SimpleActionClient()

    # Send a goal from the client
    action_client.send_goal(5)

    # Use MultiThreadedExecutor to run both nodes simultaneously
    try:
        executor = MultiThreadedExecutor()
        executor.add_node(action_server)
        executor.add_node(action_client)

        # Spin both nodes
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        action_server.destroy()
        action_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()