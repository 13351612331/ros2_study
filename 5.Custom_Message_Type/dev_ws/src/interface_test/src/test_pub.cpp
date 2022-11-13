//
// Created by kangyu on 22-11-13.
//
#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/msg/num.hpp"
#include <chrono>

using namespace std::chrono_literals;
class MinimalPublisher : public rclcpp::Node {
public:
  MinimalPublisher() : Node("minimal_publisher"), m_count(0) {
    m_publisher =
        this->create_publisher<tutorial_interfaces::msg::Num>("topic", 10);
    m_timer = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisher::TimerCallback, this));
  }

private:
  void TimerCallback() {
    auto message = tutorial_interfaces::msg::Num();
    message.num = this->m_count++;
    RCLCPP_INFO(this->get_logger(), "Publishing: '%ld'", message.num);
    m_publisher->publish(message);
  }
  size_t m_count;
  rclcpp::TimerBase::SharedPtr m_timer;
  rclcpp::Publisher<tutorial_interfaces::msg::Num>::SharedPtr m_publisher;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}