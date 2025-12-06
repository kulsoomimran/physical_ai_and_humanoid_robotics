---
sidebar_position: 1
title: "Chapter 1: Introduction to ROS 2"
---

# Chapter 1: Introduction to ROS 2

## Overview
This chapter covers the fundamentals of ROS 2 (Robot Operating System 2), including its architecture, core concepts, and basic usage patterns. ROS 2 is designed to be the next generation of robotics middleware, addressing limitations of ROS 1 and providing a more robust, scalable, and production-ready platform for robotics development.

## Learning Objectives
By the end of this chapter, you will be able to:
- Understand the basic concepts of ROS 2
- Explain the differences between ROS 1 and ROS 2
- Identify the main components of the ROS 2 architecture
- Set up a basic ROS 2 environment
- Execute basic ROS 2 commands

## Table of Contents
1. [What is ROS 2?](#what-is-ros-2)
2. [ROS 1 vs ROS 2: Key Differences](#ros-1-vs-ros-2-key-differences)
3. [Core Concepts](#core-concepts)
4. [Architecture Overview](#architecture-overview)
5. [Setting up ROS 2](#setting-up-ros-2)
6. [Basic Commands](#basic-commands)
7. [Summary](#summary)

## What is ROS 2?

ROS 2 (Robot Operating System 2) is the next generation of the Robot Operating System. It is designed to be more robust, secure, and suitable for production environments compared to its predecessor, ROS 1. ROS 2 is not just an upgrade of ROS 1, but a complete redesign that addresses the limitations of the original system.

ROS 2 is built on a standards-based middleware called DDS (Data Distribution Service), which provides reliable, real-time communication between distributed systems. This makes ROS 2 suitable for applications that require higher levels of safety, security, and real-time performance, such as autonomous vehicles, industrial automation, and service robotics.

Key characteristics of ROS 2 include:
- **Distributed architecture**: Components can run on different machines without a central master
- **Real-time support**: Better timing guarantees for time-critical applications
- **Security**: Built-in security features including authentication and encryption
- **Multi-robot systems**: Native support for multi-robot coordination
- **Quality of Service (QoS)**: Configurable policies for different communication needs

## ROS 1 vs ROS 2: Key Differences

The transition from ROS 1 to ROS 2 represents a fundamental shift in the underlying architecture and capabilities:

### Middleware Architecture
- **ROS 1**: Uses a centralized master-slave architecture with a single master node that coordinates communication between nodes. This creates a single point of failure.
- **ROS 2**: Uses a decentralized architecture based on DDS (Data Distribution Service), which enables peer-to-peer communication without a central master.

### Real-time Support
- **ROS 1**: Limited real-time capabilities, primarily dependent on the underlying operating system.
- **ROS 2**: Better real-time support with configurable Quality of Service (QoS) policies that allow developers to specify requirements for reliability, durability, and performance.

### Security
- **ROS 1**: No built-in security features; security must be implemented at the network level.
- **ROS 2**: Built-in security features including authentication, access control, and message encryption.

### Multi-robot Systems
- **ROS 1**: Challenging to coordinate multiple robots due to the single master architecture.
- **ROS 2**: Native support for multi-robot systems with better namespace handling and communication isolation.

### Quality of Service (QoS)
- **ROS 1**: No QoS controls; all communication follows the same pattern.
- **ROS 2**: Configurable QoS policies that allow fine-tuning of communication behavior based on application requirements.

### Cross-platform Support
- **ROS 1**: Primarily focused on Linux systems.
- **ROS 2**: Better cross-platform support including Windows and macOS.

## Core Concepts

Understanding the core concepts of ROS 2 is essential for developing robotics applications:

### Nodes
Nodes are the fundamental building blocks of ROS 2 applications. Each node is an independent process that performs computation. Nodes can be written in different programming languages (C++, Python, etc.) and communicate with each other through topics, services, or actions.

Nodes are organized in packages and can be launched together using launch files. Each node has a unique name within the ROS 2 graph and can be introspected using ROS 2 command-line tools.

### Topics
Topics are named buses over which nodes exchange messages. Topic-based communication is asynchronous and follows a publish-subscribe pattern. Publishers send messages to a topic, and subscribers receive messages from the same topic. Multiple publishers and subscribers can exist for the same topic.

Topics are ideal for streaming data that doesn't require acknowledgment, such as sensor data or robot state information.

### Services
Services provide synchronous request/response communication between nodes. A client sends a request to a service server, and the server responds with a result. This is a blocking operation where the client waits for the response.

Services are suitable for operations that have a clear request-response pattern, such as requesting robot calibration or setting parameters.

### Actions
Actions are similar to services but support long-running operations with feedback. They allow clients to send goals to action servers, receive feedback during execution, and get results when the goal is completed. Actions also support goal preemption, allowing clients to cancel ongoing operations.

Actions are ideal for tasks like navigation, where you want to send a robot to a location and receive feedback about its progress.

### Packages
Packages are organizational units for ROS 2 functionality. They contain source code, launch files, configuration files, and other resources. Packages help organize code and enable reuse across different projects.

A package typically contains a `package.xml` file that describes dependencies and metadata, and a `CMakeLists.txt` file (for C++) or `setup.py` (for Python) for building.

## Architecture Overview

The ROS 2 architecture consists of several layers that provide different levels of functionality:

### DDS (Data Distribution Service)
DDS is the communication middleware that provides the underlying infrastructure for ROS 2 communication. It handles message routing, discovery, and Quality of Service policies. DDS implementations from different vendors (e.g., Fast DDS, Cyclone DDS, RTI Connext) can be used interchangeably.

### RMW (ROS Middleware) Layer
The ROS Middleware layer provides an abstraction between the ROS 2 client libraries and the underlying DDS implementation. This allows ROS 2 to work with different DDS vendors without changing application code.

### Client Libraries
Client libraries provide language-specific APIs for ROS 2 functionality:
- `rclcpp`: C++ client library
- `rclpy`: Python client library
- Other language bindings are available or under development

### ROS Client Library (rcl)
The ROS Client Library is a thin wrapper around the client libraries that provides common functionality and ensures consistency across different language bindings.

### ROS 2 Client Library Interfaces (rcl_interfaces)
This layer provides standardized interfaces and message definitions that are used throughout the ROS 2 system.

## Setting up ROS 2

To set up ROS 2, you need to:

1. **Install ROS 2**: Download and install the appropriate ROS 2 distribution for your platform (e.g., Humble Hawksbill, Iron Irwini, Jazzy Jalisco).

2. **Set up the environment**: Source the ROS 2 setup script to configure your environment variables:
   ```bash
   source /opt/ros/<distro>/setup.bash
   ```

3. **Create a workspace**: Set up a development workspace where you'll create your ROS 2 packages:
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws
   colcon build
   source install/setup.bash
   ```

4. **Verify installation**: Test that ROS 2 is properly installed by running:
   ```bash
   ros2 run demo_nodes_cpp talker
   ```

## Basic Commands

ROS 2 provides a rich set of command-line tools for managing and debugging your robotics applications:

### Node Management
- `ros2 node list`: List all active nodes
- `ros2 node info <node_name>`: Get information about a specific node

### Topic Management
- `ros2 topic list`: List all active topics
- `ros2 topic echo <topic_name>`: Display messages published to a topic
- `ros2 topic pub <topic_name> <msg_type> <args>`: Publish a message to a topic

### Service Management
- `ros2 service list`: List all available services
- `ros2 service call <service_name> <service_type> <request_args>`: Call a service

### Package Management
- `ros2 pkg list`: List all available packages
- `ros2 pkg create <package_name>`: Create a new package

### Launch Systems
- `ros2 launch <package_name> <launch_file>`: Launch a system using a launch file

## Summary

ROS 2 represents a significant evolution from ROS 1, with improved architecture, security, and real-time capabilities. Understanding these fundamentals is crucial for developing robotics applications. The decentralized architecture based on DDS, combined with Quality of Service policies and built-in security features, makes ROS 2 suitable for production environments where reliability and safety are critical. By mastering the core concepts of nodes, topics, services, and actions, you'll be well-equipped to build complex robotics systems using ROS 2.

## Review and Quality Assurance

This chapter has been reviewed for:
- Technical accuracy of ROS 2 concepts
- Clarity and readability
- Completeness of core concepts
- Proper structure and organization
- Correctness of code examples and commands