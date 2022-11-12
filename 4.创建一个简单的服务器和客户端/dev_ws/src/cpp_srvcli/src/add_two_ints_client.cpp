//
// Created by kangyu on 22-11-12.
//
#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>

using namespace std::chrono_literals;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  if (argc != 3) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Usage: add two int client X Y");
    return 1;
  }

  /**
   * 首先创建一个节点，然后创建一个客户端实例。
   */
  std::shared_ptr<rclcpp::Node> node =
      rclcpp::Node::make_shared("add_two_inte_client");
  rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client =
      node->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");

  /**
   * 接下来创建一个请求数据，包括两个加数的数值。
   */
  auto request =
      std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
  request->a = atoll(argv[1]);
  request->b = atoll(argv[2]);

  /**
   * 通过while循环等待1秒钟时间，查看服务器端是否已经启动。
   */
  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                  "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "Service not available , waiting again...");
  }

  auto result = client->async_send_request(request);
  // wait for the result
  /**
   * 等待服务器端的反馈，如果用户按下Ctrl+C，也会退出，并且打印一个错误信息。
   */
  if (rclcpp::spin_until_future_complete(node, result) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->sum);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "Failed to call service add_two_ints");
  }

  rclcpp::shutdown();
  return 0;
}