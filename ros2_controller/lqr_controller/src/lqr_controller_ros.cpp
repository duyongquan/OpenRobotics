#include <algorithm>
#include <string>
#include <limits>
#include <memory>
#include <vector>
#include <utility>

#include "lqr_controller/lqr_controller_ros.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"

using std::hypot;
using std::min;
using std::max;
using std::abs;
using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;
using namespace nav2_costmap_2d;  // NOLINT
using rcl_interfaces::msg::ParameterType;

namespace ros2_controller {
namespace lqr_controller {

void LQRController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
    (void)parent;
    (void)name;
    (void)tf;
    (void)costmap_ros;
}

void LQRController::cleanup()
{

}

void LQRController::activate()
{

}

void LQRController::deactivate()
{

}

geometry_msgs::msg::TwistStamped LQRController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & speed,
  nav2_core::GoalChecker * goal_checker)
{
    (void)pose;
    (void)speed;
    (void)goal_checker;

    // populate and return message
    geometry_msgs::msg::TwistStamped cmd_vel;
    return cmd_vel;
}

void LQRController::setPlan(const nav_msgs::msg::Path & path)
{
    (void)path;
}

void LQRController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
    (void)speed_limit;
    (void)percentage;
}

}  // namespace lqr_controller
}  // namespace ros2_controller