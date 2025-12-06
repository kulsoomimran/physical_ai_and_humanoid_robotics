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
 * Debugging Node Example for Chapter 6: ROS 2 Tools and Debugging.
 *
 * This example demonstrates various debugging techniques in ROS 2 including:
 * - Proper logging usage
 * - Parameter handling
 * - Service and topic debugging
 * - Error handling and recovery
 */

#include <chrono>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"

using namespace std::chrono_literals;

class DebuggingNode : public rclcpp::Node
{
public:
  DebuggingNode()
  : Node("debugging_node_cpp")
  {
    // Declare parameters with descriptions for debugging
    rcl_interfaces::msg::ParameterDescriptor debug_level_desc;
    debug_level_desc.description = "Debug level for logging (0=info, 1=debug, 2+=more detail)";
    this->declare_parameter("debug_level", 1, debug_level_desc);

    rcl_interfaces::msg::ParameterDescriptor interval_desc;
    interval_desc.description = "Interval for publishing messages in seconds";
    this->declare_parameter("message_interval", 1.0, interval_desc);

    rcl_interfaces::msg::ParameterDescriptor error_sim_desc;
    error_sim_desc.description = "Whether to simulate errors for debugging";
    this->declare_parameter("error_simulation", false, error_sim_desc);

    // Get parameter values
    debug_level_ = this->get_parameter("debug_level").as_int();
    message_interval_ = this->get_parameter("message_interval").as_double();
    error_simulation_ = this->get_parameter("error_simulation").as_bool();

    RCLCPP_INFO(this->get_logger(), "Initialized with debug level: %d", debug_level_);
    RCLCPP_INFO(this->get_logger(), "Message interval: %.2fs", message_interval_);
    RCLCPP_INFO(this->get_logger(), "Error simulation: %s", error_simulation_ ? "true" : "false");

    // Create publisher
    publisher_ = this->create_publisher<std_msgs::msg::String>("debug_topic_cpp", 10);

    // Create subscriber
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "debug_input_cpp",
      10,
      std::bind(&DebuggingNode::topic_callback, this, std::placeholders::_1));

    // Create service server
    service_ = this->create_service<example_interfaces::srv::AddTwoInts>(
      "debug_add_two_ints_cpp",
      [this](
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
        std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response)
        {
          RCLCPP_INFO(
            this->get_logger(),
            "Received service request: %ld + %ld",
            request->a, request->b);

          // Simulate processing time for debugging
          std::this_thread::sleep_for(100ms);

          response->sum = request->a + request->b;

          RCLCPP_INFO(this->get_logger(), "Service response: %ld", response->sum);
          (void)request_header;
        });

    // Create timer
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(message_interval_),
      std::bind(&DebuggingNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Debugging node initialized successfully");
  }

private:
  void timer_callback()
  {
    try {
      auto message = std_msgs::msg::String();
      auto time_now = std::chrono::high_resolution_clock::now();
      auto time_ns = time_now.time_since_epoch().count();

      message.data = "Debug message " + std::to_string(count_++) +
                    ": time=" + std::to_string(time_ns);

      // Simulate error for debugging purposes
      if (error_simulation_ && count_ % 10 == 5) {
        RCLCPP_ERROR(this->get_logger(), "Simulated error for debugging at count %d", count_);
      }

      RCLCPP_DEBUG(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Exception in timer_callback: %s", e.what());
    }
  }

  void topic_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Subscribed: '%s'", msg->data.c_str());

    // Log with different levels based on debug level
    if (debug_level_ >= 2) {
      RCLCPP_DEBUG(this->get_logger(), "Detailed info: message length = %zu", msg->data.length());
    }
    if (debug_level_ >= 3) {
      RCLCPP_DEBUG(this->get_logger(), "Raw message: '%s'", msg->data.c_str());
    }

    // Simulate processing time for debugging
    std::this_thread::sleep_for(10ms);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service_;

  int count_ = 0;
  int debug_level_ = 1;
  double message_interval_ = 1.0;
  bool error_simulation_ = false;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<DebuggingNode>();

  RCLCPP_INFO(node->get_logger(), "Starting debugging node...");

  try {
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "Exception caught in spin: %s", e.what());
  }

  RCLCPP_INFO(node->get_logger(), "Shutting down node");
  rclcpp::shutdown();
  return 0;
}