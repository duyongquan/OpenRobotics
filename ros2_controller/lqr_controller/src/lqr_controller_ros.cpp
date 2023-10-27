#include <algorithm>
#include <string>
#include <limits>
#include <memory>
#include <vector>
#include <utility>

#include "lqr_controller/lqr_controller_ros.hpp"
#include "ros2_math/euler_angles_zxy.hpp"
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
    auto node = parent.lock();
    node_ = parent;
    if (!node) {
      throw nav2_core::PlannerException("Unable to lock node!");
    }

    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();
    tf_ = tf;
    plugin_name_ = name;
    logger_ = node->get_logger();
    clock_ = node->get_clock();

    global_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);
    last_time_ = clock_->now();

    declare_parameter_if_not_declared(node, plugin_name_ + ".max_num_iteration", rclcpp::ParameterValue(100));
    declare_parameter_if_not_declared(node, plugin_name_ + ".tolerance", rclcpp::ParameterValue(0.01));

    node->get_parameter(plugin_name_ + ".max_num_iteration", max_num_iteration_);
    node->get_parameter(plugin_name_ + ".tolerance", tolerance_);

    RCLCPP_INFO(logger_, "Initialize LQRController Success.");
}

void LQRController::cleanup()
{
  RCLCPP_INFO(
      logger_,
      "Cleaning up controller: %s of type lqr_controller::LQRController", plugin_name_.c_str());
  global_path_pub_.reset();
}

void LQRController::activate()
{
    RCLCPP_INFO(
      logger_,
      "Activating controller: %s of type lqr_controller::LQRController", plugin_name_.c_str());
    global_path_pub_->on_activate();
    // Add callback for dynamic parameters
    auto node = node_.lock();
}

void LQRController::deactivate()
{
  RCLCPP_INFO(
    logger_,
    "Deactivating controller: %s of type lqr_controller::LQRController", plugin_name_.c_str());
  global_path_pub_->on_deactivate();
}

geometry_msgs::msg::TwistStamped LQRController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & speed,
  nav2_core::GoalChecker * goal_checker)
{
    // Update for the current goal checker's state
    geometry_msgs::msg::Pose pose_tolerance;
    geometry_msgs::msg::Twist vel_tolerance;
    if (!goal_checker->getTolerances(pose_tolerance, vel_tolerance)) {
      RCLCPP_WARN(logger_, "Unable to retrieve goal checker's tolerances!");
    }

    // Update time
    rclcpp::Time now = clock_->now(); 
    double dt = (now - last_time_).seconds();
    last_time_ = now; 

    // Update yaw
    auto current_euler = ros2_common::ros2_math::EulerAnglesZXY<double>(
      pose.pose.orientation.w,
      pose.pose.orientation.x,
      pose.pose.orientation.y,
      pose.pose.orientation.z);
    double yaw = current_euler.yaw();

    auto d_euler = ros2_common::ros2_math::EulerAnglesZXY<double>(
      global_plan_.poses[index_].pose.orientation.w,
      global_plan_.poses[index_].pose.orientation.x,
      global_plan_.poses[index_].pose.orientation.y,
      global_plan_.poses[index_].pose.orientation.z);
    double d_yaw = d_euler.yaw();

    Eigen::MatrixXd state;
    state = Eigen::MatrixXd::Zero(3, 1);

    // Find the closest pose to current pose on global path
    // geometry_msgs::msg::PoseStamped closest_pose;
    // size_t index;
    // findClosestPoseAndIndex(closest_pose, index);
    double dist = nav2_util::geometry_utils::euclidean_distance(pose, global_plan_.poses[index_]);
    if (dist < 0.05 && (index_ < global_plan_.poses.size() - 1)) {
      index_++;
    }

    state(0, 0) = global_plan_.poses[index_].pose.position.x - pose.pose.position.x;
    state(1, 0) = global_plan_.poses[index_].pose.position.y - pose.pose.position.y;
    state(2, 0) = d_yaw - yaw;

    // Robot model
    Eigen::MatrixXd A(3, 3);
    Eigen::MatrixXd B(3, 2);
    Eigen::MatrixXd Q(3, 3);
    Eigen::MatrixXd R(2, 2);
    Eigen::MatrixXd K;

    A << 1.0, 0.0, 0.0,
         0.0, 1.0, 0.0,
         0.0, 0.0, 1.0;

    B << std::cos(yaw) * dt, 0,
         std::sin(yaw) * dt, 0,
         0,                  dt;

    Q << 0.05, 0,    0,
         0,    0.03, 0,
         0,    0,    0.04;
            
    R << 0.01, 0,
         0, 0.01;

    SolveLQRProblem(A, B, Q, R, tolerance_, max_num_iteration_, &K);

    // RCLCPP_INFO(logger_, "v = %lf, w = %lf", cmd_vel.twist.linear.x, cmd_vel.twist.angular.z);

    // populate and return message
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header = pose.header;
    cmd_vel.twist.linear.x = -(K * state)(0, 0);
    cmd_vel.twist.angular.z = -(K * state)(1, 0);
    RCLCPP_INFO(logger_, "v = %lf, w = %lf", cmd_vel.twist.linear.x, cmd_vel.twist.angular.z);
    return cmd_vel;
}

void LQRController::setPlan(const nav_msgs::msg::Path & path)
{
    global_plan_ = path;
}

void LQRController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
    (void)speed_limit;
    (void)percentage;
}

}  // namespace lqr_controller
}  // namespace ros2_controller

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(ros2_controller::lqr_controller::LQRController, nav2_core::Controller)
