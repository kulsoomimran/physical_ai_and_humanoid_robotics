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
 * Simple Node Example for Chapter 5: ROS 2 Packages and Launch Files.
 *
 * This example demonstrates a basic ROS 2 node that can be used in a package
 * and launched with launch files.
 */

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class SimpleTalker : public rclcpp::Node
{
public:
  SimpleTalker(const std::string & name, const std::string & topic_name)
  : Node(name)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>(topic_name, 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&SimpleTalker::timer_callback, this));
    RCLCPP_INFO(this->get_logger(), "Simple talker node initialized with name: %s on topic: %s",
                name.c_str(), topic_name.c_str());
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello from " + this->get_name() + ": " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_ = 0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimpleTalker>("simple_talker", "chatter");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}