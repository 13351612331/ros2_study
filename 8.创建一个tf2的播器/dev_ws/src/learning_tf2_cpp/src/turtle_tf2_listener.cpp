//
// Created by kangyu on 22-11-28.
//
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "turtlesim/srv/spawn.hpp"

#include <string>

using namespace std::chrono_literals;

class FrameListener : public rclcpp::Node {
public:
  FrameListener()
      : Node("turtle_tf2_frame_listener"), m_turtleSpawningServiceReady(false),
        m_turtleSpawned(false) {
    // Declare and acquire 'target_frame' parameter
    this->declare_parameter<std::string>("target_frame", "turtle1");
    this->get_parameter("target_frame", m_targetFrame);

    m_tfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    m_transformListener =
        std::make_shared<tf2_ros::TransformListener>(*m_tfBuffer);

    // Create a client to spawn a turtle
    m_spawner = this->create_client<turtlesim::srv::Spawn>("spawn");

    // Create turtle2 velocity publisher
    m_publisher =
        this->create_publisher<geometry_msgs::msg::Twist>("turtle2/cmd_vel", 1);

    // Call on timer function every second
    m_timer =
        this->create_wall_timer(1s, std::bind(&FrameListener::OnTimer, this));
  }

private:
  void OnTimer() {
    // Store frame names in variables that will be used to compute
    // transformations
    std::string fromFrameRel = m_targetFrame.c_str();
    std::string toFrameRel = "turtle2";

    if (m_turtleSpawningServiceReady) {
      if (m_turtleSpawned) {
        geometry_msgs::msg::TransformStamped transformStamped;
        // Look up for the transformation between target_frame and turtle2
        // frames and send velocity commands for turtle2 to reach target_frame
        try {
          transformStamped = m_tfBuffer->lookupTransform(
              toFrameRel, fromFrameRel, tf2::TimePoint());
        } catch (tf2::TransformException &ex) {
          RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s : %s",
                      toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
          return;
        }
        geometry_msgs::msg::Twist msg;
        static const double scaleRotationRate = 1.0;
        msg.angular.z =
            scaleRotationRate * atan2(transformStamped.transform.translation.y,
                                      transformStamped.transform.translation.x);
        static const double scaleForwardSpeed = 0.5;
        msg.linear.x = scaleForwardSpeed *
                       sqrt(pow(transformStamped.transform.translation.x, 2) +
                            pow(transformStamped.transform.translation.y, 2));

        m_publisher->publish(msg);
      } else {
        RCLCPP_INFO(this->get_logger(), "Successfully spawned");// %s",
                    //m_clientName);
        m_turtleSpawned = true;
      }
    } else {
      // Check if the service is ready
      if (m_spawner->service_is_ready()) {
        // Initialize request with turtle name and coordinates
        // Note that x , y and theta are defined as floats in
        // turtlesim/srv/Spawn
        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
        request->x = 4.0;
        request->y = 2.0;
        request->theta = 0.0;
        request->name = "turtle2";

        // Call request
        using ServiceResponseFuture = rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture;
        auto responseReceivedCallback = [this](ServiceResponseFuture future){
          auto result = future.get();
          if(strcmp(result->name.c_str() , "turtle2") == 0){
            m_turtleSpawningServiceReady = true;
          }else{
            RCLCPP_INFO(this->get_logger() , "Service callback result mismatch");
          }
        };
        auto result_ = m_spawner->async_send_request(request , responseReceivedCallback);
        //m_clientName = result_.get()->name.c_str();
      }else{
        RCLCPP_INFO(this->get_logger(), "Service is not ready");
      }
    }
  }
  // Boolean values to store the information
  // if the service for spawning turtle is available
  bool m_turtleSpawningServiceReady;
  // if the turtle was successfully spawned
  bool m_turtleSpawned;
  std::string m_targetFrame;
  const char* m_clientName;
  std::unique_ptr<tf2_ros::Buffer> m_tfBuffer;
  std::shared_ptr<tf2_ros::TransformListener> m_transformListener{nullptr};
  rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr m_spawner{nullptr};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_publisher{nullptr};
  rclcpp::TimerBase::SharedPtr m_timer{nullptr};
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrameListener>());
  rclcpp::shutdown();
  return 0;
}