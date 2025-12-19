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
 * Simple Service Example for Chapter 4: ROS 2 Services and Actions.
 *
 * This example demonstrates:
 * - Creating a ROS 2 service server in C++
 * - Creating a ROS 2 service client in C++
 * - Request-response communication pattern
 */

#include <chrono>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using namespace std::chrono_literals;

class SimpleServiceServer : public rclcpp::Node
{
public:
  SimpleServiceServer()
  : Node("simple_service_server_cpp")
  {
    // Create a service that will use the callback function to process requests
    service_ = this->create_service<example_interfaces::srv::AddTwoInts>(
      "add_two_ints_cpp",
      [this](
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
        std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response)
        {
          // Perform the addition
          response->sum = request->a + request->b;

          // Log the request and response
          RCLCPP_INFO(
            this->get_logger(),
            "Incoming request: %ld + %ld = %ld",
            request->a, request->b, response->sum);
          (void)request_header;
        });

    RCLCPP_INFO(this->get_logger(), "Service server initialized");
  }

private:
  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service_;
};

class SimpleServiceClient : public rclcpp::Node
{
public:
  SimpleServiceClient()
  : Node("simple_service_client_cpp")
  {
    // Create a client for the AddTwoInts service
    client_ = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints_cpp");

    // Wait for the service to become available
    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
    }

    // Create the request
    request_ = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
  }

  void send_request(int64_t a, int64_t b)
  {
    // Set the request parameters
    request_->a = a;
    request_->b = b;

    // Call the service asynchronously
    auto result_future = client_->async_send_request(request_);

    RCLCPP_INFO(this->get_logger(), "Sending request: %ld + %ld", a, b);

    // Wait for the result
    RCLCPP_INFO(this->get_logger(), "Waiting for result...");
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) ==
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(
        this->get_logger(),
        "Result of %ld + %ld = %ld",
        request_->a, request_->b, result_future.get()->sum);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to call service");
    }
  }

private:
  rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;
  std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // Create nodes
  auto server_node = std::make_shared<SimpleServiceServer>();
  auto client_node = std::make_shared<SimpleServiceClient>();

  // Spin the server in a separate thread
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(server_node);
  executor.add_node(client_node);

  // Send a request from the client
  std::thread client_thread([client_node]() {
    std::this_thread::sleep_for(1s);  // Give the server time to start
    client_node->send_request(10, 20);
  });

  // Spin until interrupted
  executor.spin();

  client_thread.join();
  rclcpp::shutdown();
  return 0;
}