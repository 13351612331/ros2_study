//
// Created by kangyu on 2022/10/23.
//
/**
 * 首先是包含各种头文件，比如需要用到的一些标准C++头文件。
 * rclcpp/rclcpp.hpp是ROS2中常用C++接口的头文件，使用C++编写的ROS2节点程序一定需要包含该头文件。
 * std_msgs/msg/string.hpp是ROS2中字符串消息的头文件，后边我们会周期发布一个HelloWorld
 * 的字符串消息，所以需要包含该头文件。
 */
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
/**
 * @brief This example creates a subclass of Node and uses
 * std::bind() to register a* member function as a callback from
 * the timer.
 */

/**
 * 这一行代码是创建一个节点类MinimalPublisher，从rclcpp::Node这个ROS2节点基类继承而来。
 */
class MinimalPublisher : public rclcpp::Node {
public:
  /**
   * 这是节点类MinimalPublisherd的构造函数，将count_变量初始化为0，节点名初始化为“minimal_publisher”。
   * 构造函数内先是创建了一个发布者，发布的话题名是topic，话题消息是String，保存消息的队列长度是10，
   * 然后创建了一个定时器timer_，做了一个500ms的定时，每次触发定时器后，都会运行回调函数TimerCallback。
   */
  MinimalPublisher() : Node("minimal_publisher"), count_(0) {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisher::TimerCallback, this));
  }

private:
  /**
   * TimerCallback是这里的关键，每次触发都会发布一次话题消息。
   * message中保存的字符串是Hello world加一个计数值，
   * 然后通过RCLCPP_INFO宏函数打印一次日志信息，再通过发布者的publish方法将消息发布出去。
   */
  void TimerCallback() {
    auto message = std_msgs::msg::String();
    message.data = "Hello,world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char *argv[]) {
  /*先初始化ROS2节点*/
  rclcpp::init(argc, argv);
  /*使用rclcpp::spin创建MinimalPublisher，并且进入自旋锁，当退出锁时，就会关闭节点结束*/
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}