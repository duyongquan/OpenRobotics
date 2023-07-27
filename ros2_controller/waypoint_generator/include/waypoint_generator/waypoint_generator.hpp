/*
 * @Author: duyongquan duyongquan1@xiaomi.com
 * @Date: 2023-07-27 14:08:46
 * @LastEditors: duyongquan duyongquan1@xiaomi.com
 * @LastEditTime: 2023-07-27 16:24:34
 * @FilePath: /ros2_tutorials/src/ROS2-OpenRoboticsToolkit/ros2_controller/waypoint_generator/include/waypoint_generator/waypoint_generator.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef ROS2_CONTROLLER__WAYPOINT_GENERATOR_HPP_
#define ROS2_CONTROLLER__WAYPOINT_GENERATOR_HPP_

#include <chrono>
#include <cinttypes>
#include <functional>
#include <future>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "ros2_controller_msgs/srv/path_type.hpp"

namespace ros2_controller
{
namespace waypoint_generator
{

class VehicleWaypointGenerator: public rclcpp::Node
{
public:
    using PathType = ros2_controller_msgs::srv::PathType;
    using FollowPath = nav2_msgs::action::FollowPath;
    using GoalHandleFollowPath = rclcpp_action::ClientGoalHandle<FollowPath>;

    explicit VehicleWaypointGenerator();

private:
    void HandleNavGoalMessage(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    void HandlePathTypeRequest(
        const std::shared_ptr<PathType::Request> request,
        const std::shared_ptr<PathType::Response> response);

    void DebugString(const std::string& type);

    void PublishPath(const nav_msgs::msg::Path& path);

    nav_msgs::msg::Path CreatePath(const std::string& type);

    nav_msgs::msg::Path CreateLissajousPath();

    nav_msgs::msg::Path CreateCirclePath();

    nav_msgs::msg::Path CreateRectanglePath();

    void HandleFollowPathGoalResponseCallback(GoalHandleFollowPath::SharedPtr goal_handle);
    void HandleFollowPathFeedbackCallback(GoalHandleFollowPath::SharedPtr,
        const std::shared_ptr<const FollowPath::Feedback> feedback);
    void HandleFollowPathResultCallback(const GoalHandleFollowPath::WrappedResult & result);

    rclcpp_action::Client<FollowPath>::SharedPtr follow_path_client_{nullptr};

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr nav_goal_subscription_{nullptr};
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_{nullptr};
    // rclcpp::Service<PathType>::SharedPtr path_type_server_{nullptr};

    std::string path_type_;
    std::string deault_path_type_;
};  

}  // namespace waypoint_generator
}  // namespace ros2_controller

#endif  // ROS2_CONTROLLER__WAYPOINT_GENERATOR_HPP_