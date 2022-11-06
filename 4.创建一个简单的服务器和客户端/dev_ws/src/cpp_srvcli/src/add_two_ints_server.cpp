//
// Created by kangyu on 22-11-6.
//
#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"
#include <memory>

/**
 * @brief add函数的传入参数是服务的请求和应答数据,在其中实现请求数据的求和,
 * 然后把求和结果放到应答数据中,同时在终端中打印信息.
 * @param request 服务的请求数据
 * @param response 服务的应答数据
 */
void add(
    const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
    std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response) {
  response->sum = request->a + request->b;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
              "Incoming request\na: %ld"
              "b: %ld",
              request->a, request->b);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]",
              (long int)response->sum);
}

/**
 * @brief 在main函数中主要实现以下配置：
            1.初始化ROS2的C++库
            2.创建一个叫做add_two_ints_server的节点
            3.创建一个叫做add_two_ints的服务，绑定add回调函数
            4.打印日志信息
            5.进入自旋锁，等待客户端请求
*/
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node =
      rclcpp::Node::make_shared("add_two_inits_server");
  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service =
      node->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints",
                                                                &add);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Read to add two ints.");
  rclcpp::spin(node);
  rclcpp::shutdown();
}