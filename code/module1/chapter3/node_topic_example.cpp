// Copyright 2025 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * Node and Topic Example for Chapter 3: ROS 2 Nodes and Topics.
 *
 * This example demonstrates:
 * - Creating a ROS 2 node in C++
 * - Creating publishers and subscribers
 * - Topic-based communication
 * - Proper node lifecycle management
 */

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp/qos.hpp"

using namespace std::chrono_literals;

/* Publisher Node Class */
class NodeTopicPublisher : public rclcpp::Node
{
public:
  NodeTopicPublisher()
  : Node("node_topic_publisher_cpp")
  {
    // Create a QoS profile for the publisher
    auto qos_profile = rclcpp::QoS(rclcpp::QoSInitialization::with_history_depth(10));
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

    // Create a publisher that will publish String messages to the 'chatter_cpp' topic
    publisher_ = this->create_publisher<std_msgs::msg::String>("chatter_cpp", qos_profile);

    // Create a timer that calls the timer_callback method every 500ms
    timer_ = this->create_wall_timer(
      500ms, std::bind(&NodeTopicPublisher::timer_callback, this));

    // Initialize a counter for the messages
    count_ = 0;

    // Log that the publisher node has started
    RCLCPP_INFO(this->get_logger(), "Node Topic Publisher C++ node started");
  }

private:
  void timer_callback()
  {
    // Create a String message
    auto message = std_msgs::msg::String();
    message.data = "Hello, ROS 2 C++! Message #" + std::to_string(count_++);

    // Publish the message
    publisher_->publish(message);

    // Log the message
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

/* Subscriber Node Class */
class NodeTopicSubscriber : public rclcpp::Node
{
public:
  NodeTopicSubscriber()
  : Node("node_topic_subscriber_cpp")
  {
    // Create a QoS profile for the subscriber (should match publisher)
    auto qos_profile = rclcpp::QoS(rclcpp::QoSInitialization::with_history_depth(10));
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

    // Create a subscription that will receive String messages from the 'chatter_cpp' topic
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "chatter_cpp",
      qos_profile,
      std::bind(&NodeTopicSubscriber::topic_callback, this, std::placeholders::_1));

    // Log that the subscriber node has started
    RCLCPP_INFO(this->get_logger(), "Node Topic Subscriber C++ node started");
  }

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    // Log the received message
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

/* Main function */
int main(int argc, char * argv[])
{
  // Initialize the rclcpp library
  rclcpp::init(argc, argv);

  // Create instances of both publisher and subscriber nodes
  auto publisher_node = std::make_shared<NodeTopicPublisher>();
  auto subscriber_node = std::make_shared<NodeTopicSubscriber>();

  // Use MultiThreadedExecutor to run both nodes simultaneously
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(publisher_node);
  executor.add_node(subscriber_node);

  // Spin both nodes
  try {
    executor.spin();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception caught: %s", e.what());
  }

  // Clean up: shutdown the rclcpp library
  rclcpp::shutdown();

  return 0;
}