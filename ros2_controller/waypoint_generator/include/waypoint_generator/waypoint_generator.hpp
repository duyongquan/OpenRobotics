#ifndef ROS2_CONTROLLER__WAYPOINT_GENERATOR_HPP_
#define ROS2_CONTROLLER__WAYPOINT_GENERATOR_HPP_

#include "waypoint_generator/trajectory_generation.hpp"

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

    void HandleFollowPathGoalResponseCallback(GoalHandleFollowPath::SharedPtr goal_handle);
    void HandleFollowPathFeedbackCallback(GoalHandleFollowPath::SharedPtr,
        const std::shared_ptr<const FollowPath::Feedback> feedback);
    void HandleFollowPathResultCallback(const GoalHandleFollowPath::WrappedResult & result);

    rclcpp_action::Client<FollowPath>::SharedPtr follow_path_client_{nullptr};

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr nav_goal_subscription_{nullptr};
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_{nullptr};
    rclcpp::Service<PathType>::SharedPtr path_type_server_{nullptr};

    std::shared_ptr<TrajectoryGenerator> trajectory_generator_{nullptr};

    std::string path_type_;
    std::string deault_path_type_;
    std::string global_frame_;
};  

}  // namespace waypoint_generator
}  // namespace ros2_controller

#endif  // ROS2_CONTROLLER__WAYPOINT_GENERATOR_HPP_