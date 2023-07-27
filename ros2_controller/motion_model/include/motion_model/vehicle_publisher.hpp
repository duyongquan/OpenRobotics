#ifndef ROS2_CONTROLLER__MOTION_MODEL_VEHICLE_PUBLISHER_HPP_
#define ROS2_CONTROLLER__MOTION_MODEL_VEHICLE_PUBLISHER_HPP_

#include <chrono>
#include <cinttypes>
#include <functional>
#include <future>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "gazebo_msgs/msg/model_states.hpp"
#include "gazebo_msgs/msg/link_states.hpp"

namespace ros2_controller
{
namespace motion_model
{

class VehiclePositionPublisher: public rclcpp::Node
{
public:
  explicit VehiclePositionPublisher();

private:
  void HandleGazeboPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) const;
  void HandleGazeboModelStatesPoseCallback(const gazebo_msgs::msg::ModelStates::SharedPtr msg) const;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_{nullptr};
  rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr model_states_subscription_{nullptr};

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_publisher_{nullptr};
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr rear_pose_publisher_{nullptr};
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr center_pose_publisher_{nullptr};

  // tf
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_{nullptr};
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_{nullptr};
};  

}  // namespace motion_model
}  // namespace ros2_controller

#endif  // ROS2_CONTROLLER__MOTION_MODEL_VEHICLE_PUBLISHER_HPP_