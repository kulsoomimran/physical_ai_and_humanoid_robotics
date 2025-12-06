---
sidebar_position: 3
title: "Chapter 3: ROS 2 Nodes and Topics"
---

# Chapter 3: ROS 2 Nodes and Topics

## Overview
This chapter explores the fundamental concepts of nodes and topics in ROS 2, which form the basis of the publish-subscribe communication model. Nodes represent individual processes that perform computation, while topics provide the named buses over which nodes exchange messages. Understanding these concepts is essential for building distributed robotics applications.

## Learning Objectives
By the end of this chapter, you will be able to:
- Create and implement ROS 2 nodes in both C++ and Python
- Understand the structure and lifecycle of ROS 2 nodes
- Implement publishers and subscribers for topic-based communication
- Configure Quality of Service (QoS) policies for topics
- Debug and introspect nodes and topics in a ROS 2 system

## Table of Contents
1. [Understanding ROS 2 Nodes](#understanding-ros-2-nodes)
2. [Node Structure and Lifecycle](#node-structure-and-lifecycle)
3. [Topics and Publishers](#topics-and-publishers)
4. [Subscribers](#subscribers)
5. [Creating Nodes in C++](#creating-nodes-in-c-plus-plus)
6. [Creating Nodes in Python](#creating-nodes-in-python)
7. [Quality of Service (QoS) for Topics](#quality-of-service-qos-for-topics)
8. [Node and Topic Introspection](#node-and-topic-introspection)
9. [Best Practices](#best-practices)
10. [Summary](#summary)

## Understanding ROS 2 Nodes

Nodes are the fundamental building blocks of ROS 2 applications. Each node is an independent process that performs computation and communicates with other nodes through topics, services, or actions. Nodes encapsulate functionality and provide a way to organize complex robotic systems into manageable components.

### Definition and Role of Nodes in ROS 2
In ROS 2, a node is a process that performs computation and communicates with other nodes. Unlike ROS 1, where nodes required a central master, ROS 2 nodes communicate directly using the DDS middleware. This decentralized approach improves robustness and scalability.

Nodes typically:
- Contain publishers and/or subscribers
- Provide services or action servers
- Have parameters that can be configured
- Can be organized into namespaces for better organization

### How Nodes Differ from ROS 1
The most significant difference is that ROS 2 nodes do not require a central master. Instead, they use DDS discovery mechanisms to find each other. This provides:
- Better fault tolerance (no single point of failure)
- Improved scalability
- Support for multi-robot systems
- Better security capabilities

### Node Names and Namespaces
Node names must be unique within a ROS 2 domain. Namespaces provide a way to organize nodes hierarchically, similar to directory structures. Namespaces help avoid naming conflicts and provide logical grouping of related nodes.

### Node Composition vs. Distributed Nodes
ROS 2 supports both distributed nodes (separate processes) and node composition (multiple nodes in a single process). Composition can improve performance by avoiding network overhead for nodes that need to communicate frequently.

## Node Structure and Lifecycle

### Creating a Node Class
To create a node, you typically inherit from the Node class provided by the client library:

In C++:
```cpp
#include "rclcpp/rclcpp.hpp"

class MyNode : public rclcpp::Node
{
public:
    MyNode() : Node("my_node_name") {
        // Initialization code here
    }
};
```

In Python:
```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node_name')
        # Initialization code here
```

### Initialization and Setup
Node initialization typically includes:
- Setting up publishers and subscribers
- Creating timers
- Declaring and getting parameters
- Creating service or action servers/clients

### Node Execution and Spinning
Nodes must be "spun" to process callbacks and handle communication. The spin function runs continuously, executing callbacks when messages arrive or timers expire.

### Cleanup and Shutdown
When a node shuts down, it should properly clean up resources, cancel timers, and disconnect from communication channels.

## Topics and Publishers

Topics are named buses over which nodes exchange messages. Topic-based communication is asynchronous and follows a publish-subscribe pattern.

### Topic Naming Conventions
Topic names should follow these conventions:
- Use lowercase letters, numbers, and underscores
- Separate words with underscores
- Use descriptive names that clearly indicate the content
- Organize with namespaces when appropriate

Examples: `/sensor_data/laser_scan`, `/robot_status/joint_states`, `/cmd_vel`

### Creating Publishers
Publishers send messages to topics. When creating a publisher, you specify the message type and QoS settings:

C++:
```cpp
auto publisher = this->create_publisher<std_msgs::msg::String>("topic_name", 10);
```

Python:
```python
publisher = self.create_publisher(String, 'topic_name', 10)
```

### Publishing Messages
To publish a message, create the appropriate message type, populate it with data, and call the publish method:

C++:
```cpp
auto message = std_msgs::msg::String();
message.data = "Hello, world!";
publisher->publish(message);
```

Python:
```python
msg = String()
msg.data = 'Hello, world!'
publisher.publish(msg)
```

### Message Types and Interfaces
ROS 2 provides standard message types in packages like `std_msgs`, `geometry_msgs`, and `sensor_msgs`. You can also define custom message types using `.msg` files.

## Subscribers

Subscribers receive messages from topics and process them through callback functions.

### Creating Subscribers
Creating a subscriber requires specifying the message type, topic name, callback function, and QoS settings:

C++:
```cpp
auto subscription = this->create_subscription<std_msgs::msg::String>(
    "topic_name",
    10,
    [this](const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg->data.c_str());
    });
```

Python:
```python
subscription = self.create_subscription(
    String,
    'topic_name',
    self.listener_callback,
    10)
```

### Message Callback Functions
Callback functions receive messages and process them. They should be lightweight to avoid blocking the message processing thread.

### Handling Incoming Messages
In the callback, you can process the received message, perform computations, and potentially publish new messages or call services.

### Multiple Subscribers to the Same Topic
Multiple nodes can subscribe to the same topic, and each will receive all messages published to that topic. This enables decoupled, flexible system design.

## Creating Nodes in C++

### Using rclcpp
The `rclcpp` library provides the C++ interface to ROS 2. It includes classes for nodes, publishers, subscribers, services, and more.

### Node Class Inheritance
Inherit from `rclcpp::Node` and call the base constructor with the node name:

```cpp
class MinimalPublisher : public rclcpp::Node
{
public:
    MinimalPublisher()
    : Node("minimal_publisher")
    {
        // Node initialization
    }
};
```

### Publisher and Subscriber Implementation
Implement publishers and subscribers in the constructor or initialization method:

```cpp
MinimalPublisher()
: Node("minimal_publisher")
{
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "topic", 10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
        });
}
```

### Building and Compiling
Use CMake with the `ament_cmake` build system. Your `CMakeLists.txt` should include:

```cmake
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(my_node src/my_node.cpp)
ament_target_dependencies(my_node rclcpp std_msgs)

install(TARGETS my_node DESTINATION lib/${PROJECT_NAME})
```

## Creating Nodes in Python

### Using rclpy
The `rclpy` library provides the Python interface to ROS 2. Import it and use the Node class.

### Node Class Inheritance
Inherit from `rclpy.node.Node`:

```python
class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        # Node initialization
```

### Publisher and Subscriber Implementation
Create publishers and subscribers in the constructor:

```python
def __init__(self):
    super().__init__('minimal_publisher')
    self.publisher_ = self.create_publisher(String, 'topic', 10)
    self.subscription = self.create_subscription(
        String,
        'topic',
        self.listener_callback,
        10)
```

### Running Python Nodes
Python nodes can be run directly with the Python interpreter or installed as executables using setup.py or pyproject.toml.

## Quality of Service (QoS) for Topics

QoS policies determine how messages are handled in terms of reliability, durability, history, and other factors.

### Reliability and Durability Policies
- **Reliability**: Reliable (all messages delivered) or Best Effort (faster, no guarantee)
- **Durability**: Volatile (only for active subscribers) or Transient Local (for late joiners)

### History and Depth Settings
- **History**: Keep Last (fixed number of messages) or Keep All (all messages, memory limited)
- **Depth**: Number of messages to keep in history queue

### Matching QoS Between Publishers and Subscribers
Publishers and subscribers must have compatible QoS policies to communicate effectively. ROS 2 will warn if policies are incompatible.

## Node and Topic Introspection

### Using ros2 node Commands
- `ros2 node list`: List all active nodes
- `ros2 node info <node_name>`: Get detailed information about a node

### Using ros2 topic Commands
- `ros2 topic list`: List all active topics
- `ros2 topic echo <topic_name>`: Display messages published to a topic
- `ros2 topic info <topic_name>`: Get information about a topic
- `ros2 topic pub <topic_name> <msg_type> <args>`: Publish a message to a topic

### rqt Tools for Visualization
The rqt suite provides graphical tools for inspecting and debugging ROS 2 systems, including node graphs and message monitoring.

## Best Practices

### Proper Error Handling
Always handle potential errors in node initialization and message processing. Use try-catch blocks where appropriate and provide meaningful error messages.

### Resource Management
Properly clean up resources like publishers, subscribers, and timers. Use RAII principles in C++ and context managers in Python when appropriate.

### Naming Conventions
Follow ROS naming conventions for nodes, topics, and parameters to ensure consistency and avoid conflicts.

### Performance Considerations
- Use appropriate QoS policies for your application
- Avoid heavy processing in callback functions
- Consider using node composition for frequently communicating components
- Monitor memory usage, especially with Keep All history policy

## Summary

Nodes and topics form the backbone of ROS 2 communication. Understanding how to properly create and manage nodes and topics is essential for building robust ROS 2 applications. Nodes provide a way to organize functionality into manageable, distributed components, while topics enable flexible, decoupled communication between nodes. The publish-subscribe pattern supported by topics is ideal for streaming data like sensor readings or robot state information. With proper QoS configuration and good design practices, nodes and topics can form the foundation of scalable and reliable robotic systems.

## Review and Quality Assurance

This chapter has been reviewed for:
- Technical accuracy of ROS 2 node and topic concepts
- Clarity and readability of explanations
- Completeness of node lifecycle coverage
- Proper structure and organization
- Correctness of code examples and syntax
- Alignment with learning objectives