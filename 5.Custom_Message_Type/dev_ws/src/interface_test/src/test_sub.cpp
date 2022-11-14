//
// Created by kangyu on 22-11-14.
//
#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/msg/num.hpp"

class MinimalSubscriber : public rclcpp::Node {
public:
  MinimalSubscriber() : Node("minimal_subscriber") {
    m_subscription = this->create_subscription<tutorial_interfaces::msg::Num>(
        "topic", 10,
        std::bind(&MinimalSubscriber::TopicCallback, this,
                  std::placeholders::_1));
  }

private:
  void TopicCallback(const tutorial_interfaces::msg::Num::SharedPtr msg) const {
      RCLCPP_INFO(this->get_logger() , "I heard: '%ld'" , msg->num);
  }
  rclcpp::Subscription<tutorial_interfaces::msg::Num>::SharedPtr m_subscription;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}