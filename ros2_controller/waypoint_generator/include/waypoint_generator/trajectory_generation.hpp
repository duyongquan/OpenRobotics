#ifndef ROS2_CONTROLLER__TRAJECTORY_GENERATION_HPP_
#define ROS2_CONTROLLER__TRAJECTORY_GENERATION_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"

#include <chrono>
#include <cinttypes>
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <cmath>

namespace ros2_controller {
namespace waypoint_generator {

/**
 * circle, epitrochoid, square, infinite
*/
enum class TrajectoryType
{
    Circle,
    Epitrochoid,
    Square,
    Infinite
};

class TrajectoryGenerator
{
public:
    TrajectoryGenerator();

    nav_msgs::msg::Path CreatePath(const TrajectoryType& type);

private:
    nav_msgs::msg::Path CreateCirclePath();
    nav_msgs::msg::Path CreateEpitrochoidPath();
    nav_msgs::msg::Path CreateSquarePath();
    nav_msgs::msg::Path CreateInfinitePath();

    void SetQuaternion(geometry_msgs::msg::PoseStamped& pose, const double yaw);
};

}  // namespace waypoint_generator
}  // namespace ros2_controller

#endif  // ROS2_CONTROLLER__TRAJECTORY_GENERATION_HPP_