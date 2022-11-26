//
// Created by kangyu on 22-11-26.
//

/**
 * 首先我们包含了transform_stamped.hpp这个头文件，后续会使用到其中坐标系的描述消息TransformStamped。
 * 然后还包含了tf2的两个头文件，其中tf2::Quaternion这个类会提供欧拉角和四元数的转换公式，
 * static_transform_broadcaster.h中的StaticTransformBroadcaster则是用来实例化静态坐标变化广播器的。
 */
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

class StaticFramePublisher : public rclcpp::Node {
public:
  StaticFramePublisher(char *transformation[])
      : Node("static_turtle_tf2_broadcaster") {
    m_tfPublisher = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    // Publish static transforms once at startup
    this->MakeTransforms(transformation);
  }

private:
  void MakeTransforms(char *transformation[]) {
    rclcpp::Time now;
    // 这里创建了一个TransformStamped对象，作为后续发送坐标变换信息的消息载体
    geometry_msgs::msg::TransformStamped t;

    // 时间戳：这里使用了当前的时间rclcpp::Time
    t.header.stamp = now;
    // 父坐标系：这里是world
    t.header.frame_id = "world";
    // 子坐标系：这里使用运行终端传入的坐标系名称
    t.child_frame_id = transformation[1];

    // 然后设置了海龟的姿态信息，包括平移和旋转。
    t.transform.translation.x = atof(transformation[2]);
    t.transform.translation.y = atof(transformation[3]);
    t.transform.translation.z = atof(transformation[4]);
    tf2::Quaternion q;
    q.setRPY(atof(transformation[5]), atof(transformation[6]),
             atof(transformation[7]));
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    // 最后就是将设置好的静态坐标变换信息广播出去了
    m_tfPublisher->sendTransform(t);
  }

  /**
   * 接下来，使用StaticFramePublisher的构造函数初始化节点，
   * 然后创建了一个广播器StaticTransformBroadcaster，
   * 用于广播静态的坐标变换信息。
   */
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> m_tfPublisher;
};
int main(int argc, char **argv) {
  auto logger = rclcpp::get_logger("logger");

  // Obtain parameters from command line arguments
  if (argc != 8) {
    RCLCPP_INFO(logger,
                "Invalid number of parameters\n usage: "
                "ros2 run learning_tf2_cpp static_turtle_tf2_broadcaster "
                "child_frame_name x y z roll pitch yaw");
    return 1;
  }

  // As the parent frame of the transform is 'world' , it is necessary to check
  // that the frame name passed is different
  if (strcmp(argv[1], "world") == 0) {
    RCLCPP_INFO(logger, "Your static turtls name cannot be 'world'");
    return 1;
  }

  // Pass parameters and initialize node
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StaticFramePublisher>(argv));
  rclcpp::shutdown();
  return 0;
}