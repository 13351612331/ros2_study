//
// Created by kangyu on 2022/11/1.
//
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <memory>

class MinimalSubscriber : public rclcpp::Node {
public:
  /**
   * 构造函数中创建了订阅者，订阅String消息，订阅的话题名叫做“topic”，
   * 保存消息的队列长度是10，当订阅到数据时，会进入回调函数topic_callback
   */
  MinimalSubscriber() : Node("minimal_subscribe") {
    m_subscription = this->create_subscription<std_msgs::msg::String>(
        "topic", 10, std::bind(&MinimalSubscriber::TopicCallback, this, std::placeholders::_1));
  }
  ~MinimalSubscriber() {}

private:
  /**
   * @brief 回调函数中会收到String消息，然后并没有做太多处理，只是通过RCLCPP_INFO打印出来。
   * @param msg receive msg
   */
  void TopicCallback(const std_msgs::msg::String::SharedPtr msg) const {
    RCLCPP_INFO(this->get_logger(), "I heard : '%s'", msg->data.c_str());
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_subscription;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}