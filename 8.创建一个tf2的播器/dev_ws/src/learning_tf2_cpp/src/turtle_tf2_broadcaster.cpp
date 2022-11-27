//
// Created by kangyu on 22-11-27.
//
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "turtlesim/msg/pose.hpp"
#include "tf2/LinearMath/Quaternion.h"

class FramePublisher : public rclcpp::Node {
public:
  FramePublisher() : Node("turtle_tf2_frame_publisher") {
    // Declare and acquire 'turtleName' parameter
    // 在FramePublisher类的构造函数中，首先声明了一个参数“turtleName”，
    // 并动态获取该参数的值，表示海龟的名字，比如turtle1和turtle2。
    this->declare_parameter<std::string>("turtleName", "turtle");
    this->get_parameter("turtleName", m_turtleName);

    // 接下来订阅话题turtleX/pose，并且绑定回调函数HandleTurtlePose。
    // Initialize the transform broadcaster
    m_tfBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Subscribe to a turtle{1}{2}/pose topic and call HandleTurtlePose
    // callback function on each message
    std::ostringstream stream;
    stream << "/" << m_turtleName.c_str() << "/pose";
    std::string topicName = stream.str();

    m_subscription = this->create_subscription<turtlesim::msg::Pose>(
        topicName, 10,
        std::bind(&FramePublisher::HandleTurtlePose, this,
                  std::placeholders::_1));
  }

private:
  void HandleTurtlePose(const std::shared_ptr<turtlesim::msg::Pose> msg) {
    rclcpp::Time now = this->get_clock()->now();
    geometry_msgs::msg::TransformStamped t;

    // Read message content and assign it to corresponding tf variables
    t.header.stamp = now;
    t.header.frame_id = "world";
    t.child_frame_id = m_turtleName.c_str();

    // Turtle only exists in 2D , thus we get x and y translation coordinates from the message and set the z coordinate to 0
    t.transform.translation.x = msg->x;
    t.transform.translation.y = msg->y;
    t.transform.translation.z = 0.0;

    // For the same reason ,turtle can only rotate around one axis
    // and this why we set rotation in x and y to 0 and obtain rotation in z axis from the message
    tf2::Quaternion q;
    q.setRPY(0 , 0 , msg->theta);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    // send the transformation
    m_tfBroadcaster->sendTransform(t);
  }
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr m_subscription;
  std::unique_ptr<tf2_ros::TransformBroadcaster> m_tfBroadcaster;
  std::string m_turtleName;
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FramePublisher>());
  rclcpp::shutdown();
  return 0;
}