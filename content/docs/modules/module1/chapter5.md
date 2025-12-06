---
sidebar_position: 5
title: "Chapter 5: ROS 2 Packages and Launch Files"
---

# Chapter 5: ROS 2 Packages and Launch Files

## Overview
This chapter explores the fundamental concepts of ROS 2 packages and launch files, which are essential for organizing, building, and executing ROS 2 applications. Packages provide the basic unit of organization for ROS 2 software, while launch files enable the configuration and execution of complex systems with multiple nodes.

## Learning Objectives
By the end of this chapter, you will be able to:
- Create and structure ROS 2 packages with proper metadata
- Understand package.xml and CMakeLists.txt configuration files
- Use colcon build system to compile packages
- Create launch files in XML, YAML, and Python formats
- Configure node parameters, namespaces, and remappings in launch files
- Organize complex ROS 2 systems using launch files

## Table of Contents
1. [Understanding ROS 2 Packages](#understanding-ros-2-packages)
2. [Package Structure and Organization](#package-structure-and-organization)
3. [Package Configuration Files](#package-configuration-files)
4. [The colcon Build System](#the-colcon-build-system)
5. [Creating Launch Files](#creating-launch-files)
6. [Launch File Formats (XML, YAML, Python)](#launch-file-formats-xml-yaml-python)
7. [Parameters and Namespaces in Launch Files](#parameters-and-namespaces-in-launch-files)
8. [Node Remapping and Arguments](#node-remapping-and-arguments)
9. [Advanced Launch File Features](#advanced-launch-file-features)
10. [Best Practices for Packages and Launch Files](#best-practices-for-packages-and-launch-files)
11. [Summary](#summary)

## Understanding ROS 2 Packages

### Package as the Basic Unit of Organization
ROS 2 packages serve as the fundamental unit for organizing software components. Each package typically contains:
- Source code (C++ or Python)
- Configuration files
- Launch files
- Documentation
- Tests

### Package Types
ROS 2 supports different package types based on build systems:
- **ament_cmake**: For C++ packages using CMake
- **ament_python**: For Python packages
- **ament_cargo**: For Rust packages
- **cmake**: For CMake packages that don't use ament

### Package Dependencies
Packages declare dependencies in their package.xml file:
- **buildtool_depend**: Build system dependencies (e.g., ament_cmake)
- **build_depend**: Dependencies needed for compilation
- **exec_depend**: Runtime dependencies
- **test_depend**: Dependencies needed only for tests

## Package Structure and Organization

### CMake Package Structure
A typical CMake-based package includes:
```
my_package/
├── CMakeLists.txt
├── package.xml
├── include/my_package/
├── src/
│   └── my_node.cpp
└── launch/
    └── my_launch_file.py
```

### Python Package Structure
A typical Python-based package includes:
```
my_package/
├── package.xml
├── setup.py
├── setup.cfg
├── my_package/
│   ├── __init__.py
│   └── my_node.py
└── launch/
    └── my_launch_file.py
```

### Package Naming Conventions
- Use lowercase with underscores (snake_case)
- Avoid spaces and special characters
- Be descriptive but concise
- Follow ROS naming conventions

## Package Configuration Files

### package.xml
The package.xml file contains metadata about the package:
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_package</name>
  <version>0.0.0</version>
  <description>My example ROS 2 package</description>
  <maintainer email="user@example.com">User Name</maintainer>
  <license>Apache License 2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### CMakeLists.txt
The CMakeLists.txt file describes how to build the package:
```cmake
cmake_minimum_required(VERSION 3.8)
project(my_package)

find_package(ament_cmake REQUIRED)

# Find dependencies
find_package(rclcpp REQUIRED)

# Add executable
add_executable(my_node src/my_node.cpp)
ament_target_dependencies(my_node rclcpp)

# Install targets
install(TARGETS
  my_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
```

## The colcon Build System

### Workspace Structure
A ROS 2 workspace typically has this structure:
```
ros2_ws/
├── src/
│   └── my_package/
├── build/
├── install/
└── log/
```

### Basic colcon Commands
- `colcon build`: Build all packages in the workspace
- `colcon build --packages-select <package_name>`: Build specific packages
- `colcon build --symlink-install`: Use symlinks for faster iteration
- `source install/setup.bash`: Source the built packages
- `colcon test`: Run tests for packages

### colcon Build Process
1. Scan the `src` directory for packages
2. Build each package in the `build` directory
3. Install artifacts to the `install` directory
4. Generate environment setup files

## Creating Launch Files

### Purpose of Launch Files
Launch files enable:
- Starting multiple nodes with a single command
- Configuring node parameters and arguments
- Setting up namespaces and remappings
- Managing complex system deployments

### Launch File Creation Process
1. Create launch file in the `launch/` directory
2. Define nodes and their configurations
3. Test the launch file with `ros2 launch`
4. Integrate with package build process

## Launch File Formats (XML, YAML, Python)

### XML Launch Files
XML launch files provide a declarative structure:
```xml
<launch>
  <node pkg="turtlesim" exec="turtlesim_node" name="sim"
        namespace="turtlesim1" args="--ros-args --log-level info" />
  <node pkg="turtlesim" exec="mimic" name="mimic">
    <remap from="/input/pose" to="/turtlesim1/turtle1/pose" />
  </node>
</launch>
```

### YAML Launch Files
YAML launch files offer human-readable configuration:
```yaml
launch:
  - node:
      pkg: "turtlesim"
      exec: "turtlesim_node"
      name: "sim"
      namespace: "turtlesim1"
      parameters:
        - {use_sim_time: true}
```

### Python Launch Files
Python launch files provide programmatic flexibility:
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            namespace='turtlesim1',
            executable='turtlesim_node',
            name='sim',
            arguments=['--ros-args', '--log-level', 'info']
        )
    ])
```

## Parameters and Namespaces in Launch Files

### Parameters
Parameters can be set in launch files:
- **Static parameters**: Defined at launch time
- **Dynamic parameters**: Can be changed during runtime
- **Parameter files**: YAML files containing parameter configurations

### Namespaces
Namespaces provide organization and prevent naming conflicts:
- **Global namespaces**: Apply to all nodes in the launch file
- **Node-specific namespaces**: Apply to individual nodes
- **Topic remapping**: Redirect topics to namespace-specific paths

## Node Remapping and Arguments

### Topic Remapping
Remapping allows connecting nodes with different topic names:
```python
Node(
    package='turtlesim',
    executable='mimic',
    name='mimic',
    remappings=[
        ('/input/pose', '/turtlesim1/turtle1/pose'),
        ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
    ]
)
```

### Command Line Arguments
Launch files can accept arguments:
```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch argument
    my_param = DeclareLaunchArgument(
        'my_param',
        default_value='default_value',
        description='Description of parameter'
    )

    return LaunchDescription([
        my_param,
        Node(
            package='my_package',
            executable='my_node',
            name='my_node',
            parameters=[{'my_param': LaunchConfiguration('my_param')}]
        )
    ])
```

## Advanced Launch File Features

### Conditional Launch
Launch files can include conditional logic:
- **If conditions**: Launch nodes based on parameters
- **Unless conditions**: Skip nodes based on parameters

### Event Handling
Launch files can respond to events:
- **Process exit events**: Handle node crashes or shutdowns
- **Timer events**: Schedule actions at specific times
- **Custom events**: Trigger specific behaviors

### Launch Includes
Complex systems can be modularized:
- **Include other launch files**: Reuse launch configurations
- **Pass arguments between launch files**: Share parameters
- **Namespace inheritance**: Manage nested namespaces

## Best Practices for Packages and Launch Files

### Package Best Practices
- Follow consistent naming conventions
- Document dependencies clearly
- Use appropriate build types
- Include comprehensive package descriptions
- Add tests and documentation

### Launch File Best Practices
- Keep launch files focused and modular
- Use descriptive names for nodes
- Document parameter values
- Handle errors gracefully
- Test launch files thoroughly

### Performance Considerations
- Minimize unnecessary dependencies
- Use efficient build configurations
- Optimize launch file execution
- Monitor resource usage

## Summary

ROS 2 packages and launch files form the backbone of ROS 2 application organization and execution. Packages provide the structure for organizing code, dependencies, and resources, while launch files enable the configuration and deployment of complex systems. Understanding how to create and use these components is essential for developing maintainable and scalable ROS 2 applications. The colcon build system streamlines the compilation process, while the various launch file formats (XML, YAML, Python) offer flexibility for different use cases and preferences.