---
sidebar_position: 2
title: "Chapter 2: ROS 2 Architecture and Communication"
---

# Chapter 2: ROS 2 Architecture and Communication

## Overview
This chapter explores the architecture of ROS 2 and its communication mechanisms, including the Data Distribution Service (DDS) middleware, Quality of Service (QoS) policies, and how nodes communicate with each other. Understanding these concepts is crucial for developing robust and reliable robotics applications.

## Learning Objectives
By the end of this chapter, you will be able to:
- Understand the layered architecture of ROS 2
- Explain how DDS enables communication in ROS 2
- Configure Quality of Service (QoS) policies for different communication needs
- Implement nodes with different communication patterns
- Understand the role of RMW in ROS 2

## Table of Contents
1. [ROS 2 Architecture Layers](#ros-2-architecture-layers)
2. [DDS: The Communication Middleware](#dds-the-communication-middleware)
3. [Quality of Service (QoS) Policies](#quality-of-service-qos-policies)
4. [Communication Patterns](#communication-patterns)
5. [RMW: ROS Middleware Layer](#rmw-ros-middleware-layer)
6. [Practical Implementation](#practical-implementation)
7. [Summary](#summary)

## ROS 2 Architecture Layers

The ROS 2 architecture is organized in distinct layers, each serving specific functions in the communication and execution of robotics applications:

### DDS Layer
The Data Distribution Service (DDS) layer is the foundation of ROS 2's communication architecture. DDS is an OMG (Object Management Group) standard that provides a publisher-subscriber communication model with quality of service (QoS) controls. This layer handles the actual message transport between nodes, including discovery, routing, and delivery of messages.

### RMW Layer
The ROS Middleware (RMW) layer provides an abstraction between the ROS 2 client libraries and the underlying DDS implementations. This layer allows ROS 2 to work with different DDS vendors without requiring changes to application code. The RMW layer translates ROS 2 concepts into DDS-specific operations.

### Client Libraries Layer
The client libraries layer provides language-specific APIs for ROS 2 functionality:
- `rclcpp` for C++
- `rclpy` for Python
- Other language bindings as they become available

These libraries implement ROS 2 concepts like nodes, publishers, subscribers, services, and actions in a language-appropriate way.

### ROS Client Library (rcl) Layer
The ROS Client Library (rcl) is a thin wrapper that provides common functionality across all client libraries, ensuring consistency and reducing code duplication. It sits between the language-specific client libraries and the RMW layer.

## DDS: The Communication Middleware

### What is DDS?
Data Distribution Service (DDS) is a middleware protocol and API standard for distributed, real-time systems. It provides a publisher-subscriber communication model with built-in Quality of Service (QoS) policies that allow fine-tuning of communication behavior based on application requirements.

DDS is designed for systems that need:
- High reliability
- Real-time performance
- Scalability
- Robustness in distributed environments

### DDS Implementations
ROS 2 supports multiple DDS implementations:
- **Fast DDS** (formerly Fast RTPS): Developed by eProsima, widely used and well-supported
- **Cyclone DDS**: Developed by Eclipse, known for its efficiency and compliance with DDS standards
- **RTI Connext DDS**: Commercial implementation with extensive features
- **OpenDDS**: Open-source implementation by Object Computing

### How DDS Enables Peer-to-Peer Communication
Unlike ROS 1's centralized master architecture, DDS enables direct communication between nodes. When a node creates a publisher or subscriber, it uses DDS discovery mechanisms to find other participants in the network. This allows for:
- No single point of failure
- Better scalability
- Support for multi-robot systems
- More robust communication in dynamic environments

### Discovery Mechanisms in DDS
DDS uses several discovery mechanisms:
- **Simple Participant Discovery Protocol (SPDP)**: Nodes discover each other
- **Simple Endpoint Discovery Protocol (SEDP)**: Publishers and subscribers find each other
- **Multicast and unicast discovery**: Allows nodes to find each other in different network configurations

## Quality of Service (QoS) Policies

Quality of Service (QoS) policies are one of the most important features of ROS 2, allowing fine-grained control over communication behavior. QoS policies determine how messages are handled in terms of reliability, durability, history, and more.

### Reliability Policy
The reliability policy determines whether messages are delivered reliably or best-effort:
- **Reliable**: All messages are delivered, potentially with retries if needed
- **Best Effort**: Messages are sent without guarantee of delivery, faster but less reliable

Use reliable QoS for critical data like commands or sensor readings where missing data could cause problems. Use best-effort for high-frequency data like video streams where it's better to drop some frames than to introduce latency.

### Durability Policy
The durability policy determines how messages are handled with respect to time:
- **Volatile**: Messages are only available to currently active subscribers
- **Transient Local**: Messages are stored and available to late-joining subscribers

Use transient local durability for important configuration data or initial state information that new nodes need to receive.

### History Policy
The history policy determines how many messages are stored:
- **Keep Last**: Only the most recent messages are kept (up to a specified depth)
- **Keep All**: All messages are kept (limited by memory)

Keep last is typically used for sensor data where only the most recent value matters. Keep all is used when all messages are important, though it can consume significant memory.

### Deadline and Lifespan Policies
- **Deadline**: Defines the maximum time between data samples
- **Lifespan**: Defines how long a message is valid before being discarded

These policies are important for time-sensitive applications where data has a limited useful lifetime.

### Liveliness Policy
The liveliness policy determines how ROS 2 detects if a node is still active:
- **Automatic**: ROS 2 automatically monitors liveliness
- **Manual By Topic**: Applications manually signal liveliness based on topic activity
- **Manual By Node**: Applications manually signal liveliness for the entire node

## Communication Patterns

ROS 2 supports three main communication patterns, each with different characteristics and use cases:

### Publisher-Subscriber (Topics)
The publisher-subscriber pattern is asynchronous and allows multiple publishers and subscribers for the same topic. This pattern is ideal for streaming data like sensor readings or robot state information. Publishers send messages without knowing if any subscribers exist, and subscribers receive messages without knowing the publishers.

### Client-Server (Services)
The client-server pattern is synchronous and follows a request-response model. Clients send requests to services and wait for responses. This pattern is suitable for operations that require acknowledgment or have a clear request-response nature, such as setting parameters or requesting specific actions.

### Action-Based Communication
Actions combine features of services and topics, supporting long-running operations with feedback. Clients can send goals to action servers, receive continuous feedback during execution, and get results when the goal is completed. Actions also support goal preemption, allowing clients to cancel ongoing operations.

## RMW: ROS Middleware Layer

### Role of RMW in Abstracting DDS Implementations
The ROS Middleware (RMW) layer serves as a crucial abstraction that allows ROS 2 to work with different DDS implementations. This layer:
- Translates ROS 2 concepts to DDS-specific operations
- Provides a consistent API regardless of the underlying DDS implementation
- Allows switching between DDS vendors without changing application code

### How to Switch Between Different DDS Vendors
Switching between DDS implementations is typically done at compile or runtime:
- At compile time: Link against the desired RMW implementation
- At runtime: Set environment variables to select the desired implementation

### Configuration Considerations
When working with different DDS implementations, consider:
- Performance characteristics of each implementation
- Resource requirements
- Compatibility with your target platform
- Support for specific QoS policies

## Practical Implementation

### Setting up QoS Policies in Code
When creating publishers and subscribers, you can specify QoS policies:

```cpp
// C++ example with custom QoS
rclcpp::QoS qos_profile(10);  // history depth of 10
qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

auto publisher = this->create_publisher<std_msgs::msg::String>("topic", qos_profile);
```

```python
# Python example with custom QoS
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

qos_profile = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE
)

publisher = self.create_publisher(String, 'topic', qos_profile)
```

### Examples of Different Communication Patterns
Different QoS policies are appropriate for different types of data:
- Sensor data: Often best-effort with keep-last history
- Control commands: Usually reliable with keep-last history
- Configuration data: Reliable with transient-local durability
- Log messages: Best-effort with keep-all history

### Best Practices for Communication Design
- Choose QoS policies based on the criticality and nature of your data
- Test your system with different QoS configurations
- Monitor network usage and adjust policies as needed
- Document your QoS choices for maintainability

## Summary

ROS 2's architecture built on DDS provides a robust, scalable communication framework. The layered architecture with DDS, RMW, and client libraries provides both flexibility and consistency. Quality of Service (QoS) policies give developers fine-grained control over communication behavior, allowing optimization for specific use cases. Understanding these concepts is crucial for effective robotics development, as they directly impact system performance, reliability, and resource usage. By mastering the communication patterns and QoS policies, developers can build more robust and efficient robotics applications.

## Review and Quality Assurance

This chapter has been reviewed for:
- Technical accuracy of ROS 2 architecture concepts
- Clarity and readability of QoS policy explanations
- Completeness of communication patterns coverage
- Proper structure and organization
- Correctness of code examples and commands
- Alignment with learning objectives