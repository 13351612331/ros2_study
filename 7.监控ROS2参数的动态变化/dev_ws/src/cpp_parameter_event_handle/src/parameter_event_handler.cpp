//
// Created by kangyu on 22-11-21.
//
#include "rclcpp/rclcpp.hpp"

class SampleNodeWithParameters : public rclcpp::Node {
public:
  SampleNodeWithParameters() : Node("node_with_parameters") {
    // 声明了一个整型数参数an_int_param，参数值默认是0
    this->declare_parameter("an_int_param", 0);

    // Create a parameter subscriber that can be used to monitor parameter
    // changes
    // 创建了ParameterEventHandler实例，用于监控参数的变化
    m_paramSubscriber = std::make_shared<rclcpp::ParameterEventHandler>(this);

    // Set a callback for this node's integer parameter , "an_int_param"
    // 创建了一个lambda函数，并设置为参数变化时的回调函数
    auto cb = [this](const rclcpp::Parameter &p) {
      RCLCPP_INFO(
          this->get_logger(),
          "cb:Received an update to parameter \"%s\" of type %s : \"%ld\"",
          p.get_name().c_str(), p.get_type_name().c_str(), p.as_int());
    };
    m_cbHandle = m_paramSubscriber->add_parameter_callback("an_int_param" , cb);
  }

private:
  std::shared_ptr<rclcpp::ParameterEventHandler> m_paramSubscriber;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> m_cbHandle;
};

int main(int argc , char** argv){
  rclcpp::init(argc , argv);
  rclcpp::spin(std::make_shared<SampleNodeWithParameters>());
  rclcpp::shutdown();

  return 0;
}