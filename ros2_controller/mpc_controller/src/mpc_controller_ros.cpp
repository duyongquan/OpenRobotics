// Copyright (c) 2020 Shrijit Singh
// Copyright (c) 2020 Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <string>
#include <limits>
#include <memory>
#include <vector>
#include <utility>

#include "mpc_controller/mpc_controller_ros.hpp"
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
namespace mpc_controller {

void MPCController::configure(
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

//   double transform_tolerance = 0.1;
//   double control_frequency = 20.0;
//   goal_dist_tol_ = 0.25;  // reasonable default before first update

//   declare_parameter_if_not_declared(
//     node, plugin_name_ + ".desired_linear_vel", rclcpp::ParameterValue(0.5));
//   declare_parameter_if_not_declared(
//     node, plugin_name_ + ".lookahead_dist", rclcpp::ParameterValue(0.6));
//   declare_parameter_if_not_declared(
//     node, plugin_name_ + ".min_lookahead_dist", rclcpp::ParameterValue(0.3));
//   declare_parameter_if_not_declared(
//     node, plugin_name_ + ".max_lookahead_dist", rclcpp::ParameterValue(0.9));
//   declare_parameter_if_not_declared(
//     node, plugin_name_ + ".lookahead_time", rclcpp::ParameterValue(1.5));
//   declare_parameter_if_not_declared(
//     node, plugin_name_ + ".rotate_to_heading_angular_vel", rclcpp::ParameterValue(1.8));
//   declare_parameter_if_not_declared(
//     node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(0.1));
//   declare_parameter_if_not_declared(
//     node, plugin_name_ + ".use_velocity_scaled_lookahead_dist",
//     rclcpp::ParameterValue(false));
//   declare_parameter_if_not_declared(
//     node, plugin_name_ + ".min_approach_linear_velocity", rclcpp::ParameterValue(0.05));
//   declare_parameter_if_not_declared(
//     node, plugin_name_ + ".approach_velocity_scaling_dist",
//     rclcpp::ParameterValue(0.6));
//   declare_parameter_if_not_declared(
//     node, plugin_name_ + ".max_allowed_time_to_collision_up_to_carrot",
//     rclcpp::ParameterValue(1.0));
//   declare_parameter_if_not_declared(
//     node, plugin_name_ + ".use_collision_detection",
//     rclcpp::ParameterValue(true));
//   declare_parameter_if_not_declared(
//     node, plugin_name_ + ".use_regulated_linear_velocity_scaling", rclcpp::ParameterValue(true));
//   declare_parameter_if_not_declared(
//     node, plugin_name_ + ".use_cost_regulated_linear_velocity_scaling",
//     rclcpp::ParameterValue(true));
//   declare_parameter_if_not_declared(
//     node, plugin_name_ + ".cost_scaling_dist", rclcpp::ParameterValue(0.6));
//   declare_parameter_if_not_declared(
//     node, plugin_name_ + ".cost_scaling_gain", rclcpp::ParameterValue(1.0));
//   declare_parameter_if_not_declared(
//     node, plugin_name_ + ".inflation_cost_scaling_factor", rclcpp::ParameterValue(3.0));
//   declare_parameter_if_not_declared(
//     node, plugin_name_ + ".regulated_linear_scaling_min_radius", rclcpp::ParameterValue(0.90));
//   declare_parameter_if_not_declared(
//     node, plugin_name_ + ".regulated_linear_scaling_min_speed", rclcpp::ParameterValue(0.25));
//   declare_parameter_if_not_declared(
//     node, plugin_name_ + ".use_rotate_to_heading", rclcpp::ParameterValue(true));
//   declare_parameter_if_not_declared(
//     node, plugin_name_ + ".rotate_to_heading_min_angle", rclcpp::ParameterValue(0.785));
//   declare_parameter_if_not_declared(
//     node, plugin_name_ + ".max_angular_accel", rclcpp::ParameterValue(3.2));
//   declare_parameter_if_not_declared(
//     node, plugin_name_ + ".allow_reversing", rclcpp::ParameterValue(false));
//   declare_parameter_if_not_declared(
//     node, plugin_name_ + ".max_robot_pose_search_dist",
//     rclcpp::ParameterValue(getCostmapMaxExtent()));
//   declare_parameter_if_not_declared(
//     node, plugin_name_ + ".use_interpolation",
//     rclcpp::ParameterValue(true));

//   node->get_parameter(plugin_name_ + ".desired_linear_vel", desired_linear_vel_);
//   base_desired_linear_vel_ = desired_linear_vel_;
//   node->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
//   node->get_parameter(plugin_name_ + ".min_lookahead_dist", min_lookahead_dist_);
//   node->get_parameter(plugin_name_ + ".max_lookahead_dist", max_lookahead_dist_);
//   node->get_parameter(plugin_name_ + ".lookahead_time", lookahead_time_);
//   node->get_parameter(
//     plugin_name_ + ".rotate_to_heading_angular_vel",
//     rotate_to_heading_angular_vel_);
//   node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
//   node->get_parameter(
//     plugin_name_ + ".use_velocity_scaled_lookahead_dist",
//     use_velocity_scaled_lookahead_dist_);
//   node->get_parameter(
//     plugin_name_ + ".min_approach_linear_velocity",
//     min_approach_linear_velocity_);
//   node->get_parameter(
//     plugin_name_ + ".approach_velocity_scaling_dist",
//     approach_velocity_scaling_dist_);
//   if (approach_velocity_scaling_dist_ > costmap_->getSizeInMetersX() / 2.0) {
//     RCLCPP_WARN(
//       logger_, "approach_velocity_scaling_dist is larger than forward costmap extent, "
//       "leading to permanent slowdown");
//   }
//   node->get_parameter(
//     plugin_name_ + ".max_allowed_time_to_collision_up_to_carrot",
//     max_allowed_time_to_collision_up_to_carrot_);
//   node->get_parameter(
//     plugin_name_ + ".use_collision_detection",
//     use_collision_detection_);
//   node->get_parameter(
//     plugin_name_ + ".use_regulated_linear_velocity_scaling",
//     use_regulated_linear_velocity_scaling_);
//   node->get_parameter(
//     plugin_name_ + ".use_cost_regulated_linear_velocity_scaling",
//     use_cost_regulated_linear_velocity_scaling_);
//   node->get_parameter(plugin_name_ + ".cost_scaling_dist", cost_scaling_dist_);
//   node->get_parameter(plugin_name_ + ".cost_scaling_gain", cost_scaling_gain_);
//   node->get_parameter(
//     plugin_name_ + ".inflation_cost_scaling_factor",
//     inflation_cost_scaling_factor_);
//   node->get_parameter(
//     plugin_name_ + ".regulated_linear_scaling_min_radius",
//     regulated_linear_scaling_min_radius_);
//   node->get_parameter(
//     plugin_name_ + ".regulated_linear_scaling_min_speed",
//     regulated_linear_scaling_min_speed_);
//   node->get_parameter(plugin_name_ + ".use_rotate_to_heading", use_rotate_to_heading_);
//   node->get_parameter(plugin_name_ + ".rotate_to_heading_min_angle", rotate_to_heading_min_angle_);
//   node->get_parameter(plugin_name_ + ".max_angular_accel", max_angular_accel_);
//   node->get_parameter(plugin_name_ + ".allow_reversing", allow_reversing_);
//   node->get_parameter("controller_frequency", control_frequency);
//   node->get_parameter(
//     plugin_name_ + ".max_robot_pose_search_dist",
//     max_robot_pose_search_dist_);
//   node->get_parameter(
//     plugin_name_ + ".use_interpolation",
//     use_interpolation_);

//   transform_tolerance_ = tf2::durationFromSec(transform_tolerance);
//   control_duration_ = 1.0 / control_frequency;

//   if (inflation_cost_scaling_factor_ <= 0.0) {
//     RCLCPP_WARN(
//       logger_, "The value inflation_cost_scaling_factor is incorrectly set, "
//       "it should be >0. Disabling cost regulated linear velocity scaling.");
//     use_cost_regulated_linear_velocity_scaling_ = false;
//   }

//   /** Possible to drive in reverse direction if and only if
//    "use_rotate_to_heading" parameter is set to false **/

//   if (use_rotate_to_heading_ && allow_reversing_) {
//     RCLCPP_WARN(
//       logger_, "Disabling reversing. Both use_rotate_to_heading and allow_reversing "
//       "parameter cannot be set to true. By default setting use_rotate_to_heading true");
//     allow_reversing_ = false;
//   }

//   global_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);
//   carrot_pub_ = node->create_publisher<geometry_msgs::msg::PointStamped>("lookahead_point", 1);
//   carrot_arc_pub_ = node->create_publisher<nav_msgs::msg::Path>("lookahead_collision_arc", 1);

//   // initialize collision checker and set costmap
//   collision_checker_ = std::make_unique<nav2_costmap_2d::
//       FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>(costmap_);
//   collision_checker_->setCostmap(costmap_);
}

void MPCController::cleanup()
{
  RCLCPP_INFO(
    logger_,
    "Cleaning up controller: %s of type mpc_controller::MPCController", plugin_name_.c_str());
  global_path_pub_.reset();
}

void MPCController::activate()
{
  RCLCPP_INFO(
    logger_,
    "Activating controller: %s of type mpc_controller::MPCController", plugin_name_.c_str());
  global_path_pub_->on_activate();
 
  // Add callback for dynamic parameters
  auto node = node_.lock();
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(
      &MPCController::dynamicParametersCallback,
      this, std::placeholders::_1));
}

void MPCController::deactivate()
{
  RCLCPP_INFO(
    logger_,
    "Deactivating controller: %s of type mpc_controller::MPCController",
    plugin_name_.c_str());
  global_path_pub_->on_deactivate();
  dyn_params_handler_.reset();
}

geometry_msgs::msg::TwistStamped MPCController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & speed,
  nav2_core::GoalChecker * goal_checker)
{
    (void) pose;
    (void) speed;
    (void) goal_checker;

//   std::lock_guard<std::mutex> lock_reinit(mutex_);

//   // Update for the current goal checker's state
//   geometry_msgs::msg::Pose pose_tolerance;
//   geometry_msgs::msg::Twist vel_tolerance;
//   if (!goal_checker->getTolerances(pose_tolerance, vel_tolerance)) {
//     RCLCPP_WARN(logger_, "Unable to retrieve goal checker's tolerances!");
//   } else {
//     goal_dist_tol_ = pose_tolerance.position.x;
//   }

//   // Transform path to robot base frame
//   auto transformed_plan = transformGlobalPlan(pose);

//   // Find look ahead distance and point on path and publish
//   double lookahead_dist = getLookAheadDistance(speed);

//   // Check for reverse driving
//   if (allow_reversing_) {
//     // Cusp check
//     double dist_to_cusp = findVelocitySignChange(transformed_plan);

//     // if the lookahead distance is further than the cusp, use the cusp distance instead
//     if (dist_to_cusp < lookahead_dist) {
//       lookahead_dist = dist_to_cusp;
//     }
//   }

//   auto carrot_pose = getLookAheadPoint(lookahead_dist, transformed_plan);
//   carrot_pub_->publish(createCarrotMsg(carrot_pose));

//   double linear_vel, angular_vel;

//   // Find distance^2 to look ahead point (carrot) in robot base frame
//   // This is the chord length of the circle
//   const double carrot_dist2 =
//     (carrot_pose.pose.position.x * carrot_pose.pose.position.x) +
//     (carrot_pose.pose.position.y * carrot_pose.pose.position.y);

//   // Find curvature of circle (k = 1 / R)
//   double curvature = 0.0;
//   if (carrot_dist2 > 0.001) {
//     curvature = 2.0 * carrot_pose.pose.position.y / carrot_dist2;
//   }

//   // Setting the velocity direction
//   double sign = 1.0;
//   if (allow_reversing_) {
//     sign = carrot_pose.pose.position.x >= 0.0 ? 1.0 : -1.0;
//   }

//   linear_vel = desired_linear_vel_;

//   // Make sure we're in compliance with basic constraints
//   double angle_to_heading;
//   if (shouldRotateToGoalHeading(carrot_pose)) {
//     double angle_to_goal = tf2::getYaw(transformed_plan.poses.back().pose.orientation);
//     rotateToHeading(linear_vel, angular_vel, angle_to_goal, speed);
//   } else if (shouldRotateToPath(carrot_pose, angle_to_heading)) {
//     rotateToHeading(linear_vel, angular_vel, angle_to_heading, speed);
//   } else {
//     applyConstraints(
//       curvature, speed,
//       costAtPose(pose.pose.position.x, pose.pose.position.y), transformed_plan,
//       linear_vel, sign);

//     // Apply curvature to angular velocity after constraining linear velocity
//     angular_vel = linear_vel * curvature;
//   }

//   // Collision checking on this velocity heading
//   const double & carrot_dist = hypot(carrot_pose.pose.position.x, carrot_pose.pose.position.y);
//   if (use_collision_detection_ && isCollisionImminent(pose, linear_vel, angular_vel, carrot_dist)) {
//     throw nav2_core::PlannerException("RegulatedPurePursuitController detected collision ahead!");
//   }

//   // populate and return message
  geometry_msgs::msg::TwistStamped cmd_vel;
//   cmd_vel.header = pose.header;
//   cmd_vel.twist.linear.x = linear_vel;
//   cmd_vel.twist.angular.z = angular_vel;
  return cmd_vel;
}

void MPCController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
}

void MPCController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
    (void) speed_limit;
    (void) percentage;
//   if (speed_limit == nav2_costmap_2d::NO_SPEED_LIMIT) {
//     // Restore default value
//     desired_linear_vel_ = base_desired_linear_vel_;
//   } else {
//     if (percentage) {
//       // Speed limit is expressed in % from maximum speed of robot
//       desired_linear_vel_ = base_desired_linear_vel_ * speed_limit / 100.0;
//     } else {
//       // Speed limit is expressed in absolute value
//       desired_linear_vel_ = speed_limit;
//     }
//   }
}

nav_msgs::msg::Path MPCController::transformGlobalPlan(
  const geometry_msgs::msg::PoseStamped & pose)
{
    (void) pose;
//   if (global_plan_.poses.empty()) {
//     throw nav2_core::PlannerException("Received plan with zero length");
//   }

//   // let's get the pose of the robot in the frame of the plan
//   geometry_msgs::msg::PoseStamped robot_pose;
//   if (!transformPose(global_plan_.header.frame_id, pose, robot_pose)) {
//     throw nav2_core::PlannerException("Unable to transform robot pose into global plan's frame");
//   }

//   // We'll discard points on the plan that are outside the local costmap
//   double max_costmap_extent = getCostmapMaxExtent();

//   auto closest_pose_upper_bound =
//     nav2_util::geometry_utils::first_after_integrated_distance(
//     global_plan_.poses.begin(), global_plan_.poses.end(), max_robot_pose_search_dist_);

//   // First find the closest pose on the path to the robot
//   // bounded by when the path turns around (if it does) so we don't get a pose from a later
//   // portion of the path
//   auto transformation_begin =
//     nav2_util::geometry_utils::min_by(
//     global_plan_.poses.begin(), closest_pose_upper_bound,
//     [&robot_pose](const geometry_msgs::msg::PoseStamped & ps) {
//       return euclidean_distance(robot_pose, ps);
//     });

//   // Find points up to max_transform_dist so we only transform them.
//   auto transformation_end = std::find_if(
//     transformation_begin, global_plan_.poses.end(),
//     [&](const auto & pose) {
//       return euclidean_distance(pose, robot_pose) > max_costmap_extent;
//     });

//   // Lambda to transform a PoseStamped from global frame to local
//   auto transformGlobalPoseToLocal = [&](const auto & global_plan_pose) {
//       geometry_msgs::msg::PoseStamped stamped_pose, transformed_pose;
//       stamped_pose.header.frame_id = global_plan_.header.frame_id;
//       stamped_pose.header.stamp = robot_pose.header.stamp;
//       stamped_pose.pose = global_plan_pose.pose;
//       transformPose(costmap_ros_->getBaseFrameID(), stamped_pose, transformed_pose);
//       transformed_pose.pose.position.z = 0.0;
//       return transformed_pose;
//     };

//   // Transform the near part of the global plan into the robot's frame of reference.
  nav_msgs::msg::Path transformed_plan;
//   std::transform(
//     transformation_begin, transformation_end,
//     std::back_inserter(transformed_plan.poses),
//     transformGlobalPoseToLocal);
//   transformed_plan.header.frame_id = costmap_ros_->getBaseFrameID();
//   transformed_plan.header.stamp = robot_pose.header.stamp;

//   // Remove the portion of the global plan that we've already passed so we don't
//   // process it on the next iteration (this is called path pruning)
//   global_plan_.poses.erase(begin(global_plan_.poses), transformation_begin);
//   global_path_pub_->publish(transformed_plan);

//   if (transformed_plan.poses.empty()) {
//     throw nav2_core::PlannerException("Resulting plan has 0 poses in it.");
//   }

  return transformed_plan;
}


bool MPCController::transformPose(
  const std::string frame,
  const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose) const
{
    (void) frame;
    (void) in_pose;
    (void) out_pose;
//   if (in_pose.header.frame_id == frame) {
//     out_pose = in_pose;
//     return true;
//   }

//   try {
//     tf_->transform(in_pose, out_pose, frame, transform_tolerance_);
//     out_pose.header.frame_id = frame;
//     return true;
//   } catch (tf2::TransformException & ex) {
//     RCLCPP_ERROR(logger_, "Exception in transformPose: %s", ex.what());
//   }
  return false;
}


rcl_interfaces::msg::SetParametersResult MPCController::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
    (void) parameters;

  rcl_interfaces::msg::SetParametersResult result;
//   std::lock_guard<std::mutex> lock_reinit(mutex_);

//   for (auto parameter : parameters) {
//     const auto & type = parameter.get_type();
//     const auto & name = parameter.get_name();

//     if (type == ParameterType::PARAMETER_DOUBLE) {
//       if (name == plugin_name_ + ".inflation_cost_scaling_factor") {
//         if (parameter.as_double() <= 0.0) {
//           RCLCPP_WARN(
//             logger_, "The value inflation_cost_scaling_factor is incorrectly set, "
//             "it should be >0. Ignoring parameter update.");
//           continue;
//         }
//         inflation_cost_scaling_factor_ = parameter.as_double();
//       } else if (name == plugin_name_ + ".desired_linear_vel") {
//         desired_linear_vel_ = parameter.as_double();
//         base_desired_linear_vel_ = parameter.as_double();
//       } else if (name == plugin_name_ + ".lookahead_dist") {
//         lookahead_dist_ = parameter.as_double();
//       } else if (name == plugin_name_ + ".max_lookahead_dist") {
//         max_lookahead_dist_ = parameter.as_double();
//       } else if (name == plugin_name_ + ".min_lookahead_dist") {
//         min_lookahead_dist_ = parameter.as_double();
//       } else if (name == plugin_name_ + ".lookahead_time") {
//         lookahead_time_ = parameter.as_double();
//       } else if (name == plugin_name_ + ".rotate_to_heading_angular_vel") {
//         rotate_to_heading_angular_vel_ = parameter.as_double();
//       } else if (name == plugin_name_ + ".min_approach_linear_velocity") {
//         min_approach_linear_velocity_ = parameter.as_double();
//       } else if (name == plugin_name_ + ".max_allowed_time_to_collision_up_to_carrot") {
//         max_allowed_time_to_collision_up_to_carrot_ = parameter.as_double();
//       } else if (name == plugin_name_ + ".cost_scaling_dist") {
//         cost_scaling_dist_ = parameter.as_double();
//       } else if (name == plugin_name_ + ".cost_scaling_gain") {
//         cost_scaling_gain_ = parameter.as_double();
//       } else if (name == plugin_name_ + ".regulated_linear_scaling_min_radius") {
//         regulated_linear_scaling_min_radius_ = parameter.as_double();
//       } else if (name == plugin_name_ + ".transform_tolerance") {
//         double transform_tolerance = parameter.as_double();
//         transform_tolerance_ = tf2::durationFromSec(transform_tolerance);
//       } else if (name == plugin_name_ + ".regulated_linear_scaling_min_speed") {
//         regulated_linear_scaling_min_speed_ = parameter.as_double();
//       } else if (name == plugin_name_ + ".max_angular_accel") {
//         max_angular_accel_ = parameter.as_double();
//       } else if (name == plugin_name_ + ".rotate_to_heading_min_angle") {
//         rotate_to_heading_min_angle_ = parameter.as_double();
//       }
//     } else if (type == ParameterType::PARAMETER_BOOL) {
//       if (name == plugin_name_ + ".use_velocity_scaled_lookahead_dist") {
//         use_velocity_scaled_lookahead_dist_ = parameter.as_bool();
//       } else if (name == plugin_name_ + ".use_regulated_linear_velocity_scaling") {
//         use_regulated_linear_velocity_scaling_ = parameter.as_bool();
//       } else if (name == plugin_name_ + ".use_cost_regulated_linear_velocity_scaling") {
//         use_cost_regulated_linear_velocity_scaling_ = parameter.as_bool();
//       } else if (name == plugin_name_ + ".use_rotate_to_heading") {
//         if (parameter.as_bool() && allow_reversing_) {
//           RCLCPP_WARN(
//             logger_, "Both use_rotate_to_heading and allow_reversing "
//             "parameter cannot be set to true. Rejecting parameter update.");
//           continue;
//         }
//         use_rotate_to_heading_ = parameter.as_bool();
//       } else if (name == plugin_name_ + ".allow_reversing") {
//         if (use_rotate_to_heading_ && parameter.as_bool()) {
//           RCLCPP_WARN(
//             logger_, "Both use_rotate_to_heading and allow_reversing "
//             "parameter cannot be set to true. Rejecting parameter update.");
//           continue;
//         }
//         allow_reversing_ = parameter.as_bool();
//       }
//     }
//   }

//   result.successful = true;
  return result;
}

}  // namespace mpc_controller
}  // namespace ros2_controller

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(ros2_controller::mpc_controller::MPCController, nav2_core::Controller)
