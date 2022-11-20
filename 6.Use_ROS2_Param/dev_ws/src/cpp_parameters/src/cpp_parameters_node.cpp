//
// Created by kangyu on 22-11-20.
//
#include "rclcpp/rclcpp.hpp"
#include <chrono>

using namespace std::chrono_literals;

class ParametersClass : public rclcpp::Node {
public:
  ParametersClass() : Node("parameter_node") {
    /**
     * 这段代码是ParametersClass类的构造函数，其中第一行就创建了一个ROS参数，
     * 参数名为my_parameter，参数值为“world”，数据类型是string
     */
    this->declare_parameter<std::string>("my_parameter", "world");
    /**
     * 创建了一个1s周期的定时器，触发respond函数。
     */
    m_timer = this->create_wall_timer(
        1000ms, std::bind(&ParametersClass::respond, this));
  }
  /**
   * 在respond函数中，第一行会查询“my_parameter”参数，
   * 并且保存在parameter_string_这个私有变量中，
   * 紧接着第二行就把查询到的参数值打印出来了。
   */
  void respond() {
    this->get_parameter("my_parameter", m_parameterString);
    RCLCPP_INFO(this->get_logger(), "Hello %s", m_parameterString.c_str());
  }

private:
  std::string m_parameterString;
  rclcpp::TimerBase::SharedPtr m_timer;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ParametersClass>());
  rclcpp::shutdown();
  return 0;
}