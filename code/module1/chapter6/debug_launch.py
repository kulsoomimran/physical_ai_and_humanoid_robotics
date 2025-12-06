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
Debug Launch File Example for Chapter 6: ROS 2 Tools and Debugging.

This example demonstrates launch files for debugging including:
- Parameter configuration
- Multiple node launching
- Debug vs production configurations
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments for debugging configurations
    debug_level_arg = DeclareLaunchArgument(
        'debug_level',
        default_value='2',
        description='Debug level for nodes (0=info, 1=debug, 2+=more detail)'
    )

    error_simulation_arg = DeclareLaunchArgument(
        'error_simulation',
        default_value='false',
        description='Whether to simulate errors for debugging'
    )

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='debug',
        description='Namespace for the nodes'
    )

    # Get launch configurations
    debug_level = LaunchConfiguration('debug_level')
    error_simulation = LaunchConfiguration('error_simulation')
    namespace = LaunchConfiguration('namespace')

    # Create launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(debug_level_arg)
    ld.add_action(error_simulation_arg)
    ld.add_action(namespace_arg)

    # Add environment variable for logging
    log_config = SetEnvironmentVariable(
        name='RCUTILS_LOGGING_SEVERITY_THRESHOLD',
        value='DEBUG'
    )
    ld.add_action(log_config)

    # Add debugging node
    debugging_node = Node(
        package='chapter6_examples',  # This would be the actual package name
        executable='debugging_node',
        name='debugging_node',
        namespace=namespace,
        parameters=[
            {
                'debug_level': debug_level,
                'message_interval': 1.0,
                'error_simulation': error_simulation
            }
        ],
        remappings=[
            ('debug_topic', 'debug_topic'),
            ('debug_input', 'debug_input'),
        ],
        # Set output to screen for debugging
        output='screen',
        # Additional debugging flags can be added here
        arguments=['--ros-args', '--log-level', 'debug']
    )

    # Add a second instance with different parameters for comparison
    debugging_node2 = Node(
        package='chapter6_examples',
        executable='debugging_node',
        name='debugging_node2',
        namespace=namespace,
        parameters=[
            {
                'debug_level': debug_level,
                'message_interval': 2.0,  # Different interval
                'error_simulation': 'false'  # No error simulation
            }
        ],
        remappings=[
            ('debug_topic', 'debug_topic2'),
            ('debug_input', 'debug_input2'),
        ],
        output='screen',
        arguments=['--ros-args', '--log-level', 'info']
    )

    # Add nodes to launch description
    ld.add_action(debugging_node)
    ld.add_action(debugging_node2)

    return ld