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
Python Launch File Example for Chapter 5: ROS 2 Packages and Launch Files.

This example demonstrates a Python launch file that starts multiple nodes
with parameters, namespaces, and remappings.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='robot1',
        description='Namespace for the nodes'
    )

    # Get the launch configuration
    namespace = LaunchConfiguration('namespace')

    # Create launch description
    ld = LaunchDescription()

    # Add the namespace argument
    ld.add_action(namespace_arg)

    # Add simple talker node
    talker_node = Node(
        package='simple_package',
        executable='simple_talker',
        name='simple_talker',
        namespace=namespace,
        parameters=[
            # Add any parameters here if needed
        ],
        remappings=[
            # Add any remappings here if needed
        ],
        output='screen'
    )

    # Add simple listener node
    listener_node = Node(
        package='simple_package',
        executable='simple_listener',
        name='simple_listener',
        namespace=namespace,
        parameters=[
            # Add any parameters here if needed
        ],
        remappings=[
            # Remap the chatter topic to the namespaced topic
            ('chatter', 'chatter'),
        ],
        output='screen'
    )

    # Add nodes to launch description
    ld.add_action(talker_node)
    ld.add_action(listener_node)

    return ld