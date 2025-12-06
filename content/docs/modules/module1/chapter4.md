---
sidebar_position: 4
title: "Chapter 4: ROS 2 Services and Actions"
---

# Chapter 4: ROS 2 Services and Actions

## Overview
This chapter explores the service and action communication patterns in ROS 2, which complement the topic-based publish-subscribe model. Services provide synchronous request-response communication, while actions support long-running operations with feedback and goal preemption. Understanding these patterns is crucial for designing effective robotics applications that require different communication semantics than simple topic-based messaging.

## Learning Objectives
By the end of this chapter, you will be able to:
- Implement ROS 2 services for request-response communication
- Create and use actions for long-running operations
- Understand when to use services vs. actions vs. topics
- Handle service and action callbacks properly
- Configure action goals, feedback, and results

## Table of Contents
1. [Understanding Services in ROS 2](#understanding-services-in-ros-2)
2. [Service Architecture and Implementation](#service-architecture-and-implementation)
3. [Creating Services in C++](#creating-services-in-c-plus-plus)
4. [Creating Services in Python](#creating-services-in-python)
5. [Understanding Actions in ROS 2](#understanding-actions-in-ros-2)
6. [Action Architecture and Implementation](#action-architecture-and-implementation)
7. [Creating Actions in C++](#creating-actions-in-c-plus-plus)
8. [Creating Actions in Python](#creating-actions-in-python)
9. [Service and Action Best Practices](#service-and-action-best-practices)
10. [Comparing Communication Patterns](#comparing-communication-patterns)
11. [Summary](#summary)

## Understanding Services in ROS 2

Services in ROS 2 provide a request-response communication pattern, which is fundamentally different from the asynchronous, publish-subscribe model used by topics. In a service-based communication, a client sends a request to a service server, and the server processes the request and returns a response. This pattern is synchronous, meaning the client waits for the response before continuing.

### Synchronous Request-Response Pattern
The service communication pattern is characterized by:
- **Request**: Sent by the client to the service server
- **Response**: Returned by the server to the client
- **Blocking**: The client typically waits for the response before proceeding
- **One-to-one**: A single request generates a single response

### Client-Server Architecture
In ROS 2 service architecture:
- **Service Server**: Implements the service logic and responds to requests
- **Service Client**: Sends requests to the server and receives responses
- **Service Interface**: Defines the structure of requests and responses using `.srv` files

### When to Use Services vs. Topics
Use services when:
- You need guaranteed delivery and response
- The operation has a clear request-response nature
- The client needs to wait for the result before proceeding
- You need to perform one-time operations like configuration or triggering actions

Use topics when:
- You need to stream continuous data
- You don't need guaranteed delivery
- Multiple subscribers need the same data
- You want decoupled, asynchronous communication

### Service Interfaces and Message Types
Service interfaces are defined in `.srv` files that specify both the request and response message types. For example, an `AddTwoInts.srv` file might contain:

```
int64 a
int64 b
---
int64 sum
```

The part above the `---` defines the request message, and the part below defines the response message.

## Service Architecture and Implementation

### Service Definition Files (.srv)
Service definition files define the interface between service clients and servers. These files are placed in the `srv/` directory of a ROS 2 package and follow a specific format:

```
# Request message fields
field_type field_name
field_type field_name
---
# Response message fields
field_type field_name
field_type field_name
```

Each service definition consists of two parts separated by `---`. The first part defines the request message fields, and the second part defines the response message fields.

### Service Servers and Clients
- **Service Server**: Contains the implementation of the service logic. It registers a callback function that is invoked when a request arrives.
- **Service Client**: Makes requests to the service server and receives responses. It handles the communication details transparently.

### Error Handling in Services
Service implementations should handle potential errors gracefully:
- Invalid request parameters
- Service unavailability
- Network timeouts
- Internal processing errors

### Timeout and Retry Mechanisms
Clients can implement timeout and retry mechanisms to handle service unavailability:
- Setting timeouts for service calls
- Implementing retry logic with exponential backoff
- Providing fallback behavior when services are unavailable

## Creating Services in C++

### Using rclcpp for Services
The `rclcpp` library provides the C++ interface for creating services. Key classes include:
- `rclcpp::Service`: Represents a service server
- `rclcpp::Client`: Represents a service client
- Service type-specific classes generated from `.srv` files

### Defining Service Callbacks
Service callbacks receive a request and return a response:

```cpp
void handle_add_two_ints(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
  std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response)
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld, b: %ld",
    request->a, request->b);
  response->sum = request->a + request->b;
}
```

### Creating Service Servers and Clients
Service server:
```cpp
auto service = this->create_service<example_interfaces::srv::AddTwoInts>(
  "add_two_ints", std::bind(&MinimalService::handle_add_two_ints, this,
  std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
```

Service client:
```cpp
auto client = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
```

### Building and Compiling Service Nodes
In your `CMakeLists.txt`, you need to:
- Find the package containing the service definition
- Generate the service messages
- Link against the necessary libraries

```cmake
find_package(example_interfaces REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/AddTwoInts.srv"
  DEPENDENCIES builtin_interfaces
)
ament_target_dependencies(your_node_name
  rclcpp
  example_interfaces
)
```

## Creating Services in Python

### Using rclpy for Services
The `rclpy` library provides the Python interface for creating services. Key components include:
- `rclpy.service.Service`: Represents a service server
- `rclpy.client.Client`: Represents a service client
- Service type-specific classes generated from `.srv` files

### Defining Service Callbacks
In Python, service callbacks are simpler:

```python
def add_two_ints_callback(self, request, response):
    response.sum = request.a + request.b
    self.get_logger().info(f'Request received: {request.a} + {request.b} = {response.sum}')
    return response
```

### Creating Service Servers and Clients
Service server:
```python
self.srv = self.create_service(
  AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
```

Service client:
```python
self.cli = self.create_client(AddTwoInts, 'add_two_ints')
```

### Running Python Service Nodes
Python service nodes can be run directly or registered as executables in the package manifest.

## Understanding Actions in ROS 2

Actions in ROS 2 provide a communication pattern for long-running operations that may require feedback during execution and the ability to preempt or cancel the operation. Unlike services, which provide immediate responses, actions are designed for operations that may take significant time to complete.

### Long-Running Operations with Feedback
Actions are ideal for operations that:
- Take a considerable amount of time to complete
- Benefit from providing feedback during execution
- May need to be canceled or preempted
- Require goal-specific parameters

### Goal, Feedback, and Result Concepts
Actions involve three key message types:
- **Goal**: Specifies what the action should accomplish
- **Feedback**: Provides status updates during execution
- **Result**: Contains the final outcome of the action

### When to Use Actions vs. Services
Use actions when:
- The operation takes a significant amount of time
- You need feedback during execution
- The operation may be preempted or canceled
- You need to track progress of the operation

Use services when:
- The operation completes quickly
- You need a simple request-response pattern
- Preemption or feedback is not needed

### Action Interfaces and Message Types
Action interfaces are defined in `.action` files that specify the goal, result, and feedback message types:

```
# Goal definition
int32 order
---
# Result definition
int32[] sequence
---
# Feedback definition
int32[] sequence
```

## Action Architecture and Implementation

### Action Definition Files (.action)
Action definition files define the interface for actions and consist of three parts separated by `---`:
- **Goal**: Defines the parameters for initiating the action
- **Result**: Defines the output of a completed action
- **Feedback**: Defines the ongoing status updates

### Action Servers and Clients
- **Action Server**: Implements the action logic, manages goals, provides feedback, and returns results
- **Action Client**: Sends goals to the server, receives feedback, and retrieves results

### Goal Preemption and Cancellation
Actions support two ways to interrupt execution:
- **Preemption**: Replace the current goal with a new one
- **Cancellation**: Stop the current goal execution entirely

### Feedback Mechanisms
Action servers continuously send feedback messages to clients during execution, allowing clients to monitor progress and make decisions based on the ongoing status.

## Creating Actions in C++

### Using rclcpp for Actions
The `rclcpp` library provides action support through:
- `rclcpp_action::Server`: Represents an action server
- `rclcpp_action::Client`: Represents an action client
- Action type-specific classes generated from `.action` files

### Defining Action Callbacks
Action callbacks handle goal acceptance, execution, and cancellation:

```cpp
rclcpp_action::GoalResponse handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const Fibonacci::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse handle_cancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<Fibonacci>> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Fibonacci>> goal_handle)
{
  using namespace std::placeholders;
  // This needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&MinimalActionServer::execute, this, _1), goal_handle}.detach();
}
```

### Creating Action Servers and Clients
Action server:
```cpp
this->action_server_ = rclcpp_action::create_server<Fibonacci>(
  this->get_node_base_interface(),
  this->get_node_clock_interface(),
  this->get_node_logging_interface(),
  this->get_node_waitables_interface(),
  "fibonacci",
  std::bind(&MinimalActionServer::handle_goal, this, _1, _2),
  std::bind(&MinimalActionServer::handle_cancel, this, _1),
  std::bind(&MinimalActionServer::handle_accepted, this, _1));
```

Action client:
```cpp
this->action_client_ = rclcpp_action::create_client<Fibonacci>(this, "fibonacci");
```

### Handling Goals, Feedback, and Results
Action servers implement the complete execution flow:
- Accepting or rejecting goals
- Executing the action with periodic feedback
- Returning results upon completion
- Handling cancellations appropriately

## Creating Actions in Python

### Using rclpy for Actions
The `rclpy` library provides action support through:
- `rclpy.action.Server`: Represents an action server
- `rclpy.action.Client`: Represents an action client
- Action type-specific classes generated from `.action` files

### Defining Action Callbacks
Action callbacks in Python are similar to C++ but with a more Pythonic approach:

```python
async def execute_callback(self, goal_handle):
  self.get_logger().info('Executing goal...')

  feedback_msg = Fibonacci.Feedback()
  feedback_msg.sequence = [0, 1]

  for i in range(1, goal_handle.request.order):
      if goal_handle.is_cancel_requested:
          goal_handle.canceled()
          self.get_logger().info('Goal canceled')
          return Fibonacci.Result()

      feedback_msg.sequence.append(feedback_msg.sequence[i] + feedback_msg.sequence[i-1])
      self.get_logger().info(f'Publishing feedback: {feedback_msg.sequence}')

      goal_handle.publish_feedback(feedback_msg)
      await asyncio.sleep(1)  # Simulate work

  goal_handle.succeed()
  result = Fibonacci.Result()
  result.sequence = feedback_msg.sequence
  self.get_logger().info(f'Returning result: {result.sequence}')

  return result
```

### Creating Action Servers and Clients
Action server:
```python
self._action_server = ActionServer(
  self, Fibonacci, 'fibonacci', self.execute_callback)
```

Action client:
```python
self._action_client = ActionClient(self, Fibonacci, 'fibonacci')
```

### Handling Goals, Feedback, and Results
Python action clients and servers handle the same concepts as C++ versions but with Python's async/await patterns for non-blocking operations.

## Service and Action Best Practices

### Proper Error Handling
- Always handle potential service and action failures
- Implement timeouts to prevent indefinite waiting
- Provide meaningful error messages to clients
- Gracefully handle cancellation requests in actions

### Resource Management
- Properly clean up service and action handles
- Use RAII principles in C++ to manage resources
- Monitor resource usage for long-running actions
- Implement proper shutdown procedures

### Naming Conventions
- Use descriptive names for services and actions
- Follow ROS naming conventions (lowercase with underscores)
- Organize services and actions logically within packages
- Document the purpose and usage of each service/action

### Performance Considerations
- Avoid heavy processing in service callbacks to prevent blocking
- Use appropriate QoS settings for service communication
- Monitor the frequency of action feedback messages
- Consider using services for quick operations and actions for long-running ones

## Comparing Communication Patterns

### Topics vs. Services vs. Actions
- **Topics**: Asynchronous, publish-subscribe, continuous data streaming
- **Services**: Synchronous, request-response, quick operations
- **Actions**: Asynchronous with feedback, long-running operations with control

### When to Use Each Pattern
- **Topics**: Sensor data, robot state, continuous monitoring
- **Services**: Configuration changes, triggering events, quick computations
- **Actions**: Navigation, trajectory execution, complex manipulations

### Performance Trade-offs
- **Topics**: Low latency, high throughput, no guarantees
- **Services**: Guaranteed delivery, synchronous, moderate latency
- **Actions**: Complex state management, feedback capability, higher overhead

### Design Considerations
Choose the appropriate communication pattern based on:
- Operation duration
- Need for feedback
- Importance of guarantees
- Frequency of communication
- Complexity of the interaction

## Summary

Services and actions provide important communication patterns that complement topics in ROS 2. Services offer synchronous request-response communication, which is ideal for operations that require guaranteed delivery and immediate results. Actions enable long-running operations with feedback and control, making them perfect for tasks that take significant time to complete and need to provide progress updates. Understanding when and how to use each communication pattern is crucial for designing effective and efficient ROS 2 applications. By choosing the appropriate pattern for each use case, developers can create robust and responsive robotics systems.