#include "cubic_spline_planner/cubic_spline_planner.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/duration.hpp"
#include "nav2_navfn_planner/navfn.hpp"
#include "nav2_util/costmap.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

using namespace std::chrono_literals;
using nav2_util::declare_parameter_if_not_declared;
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace ros2_tutorials
{
namespace path_planner
{

CubicSplinePlanner::CubicSplinePlanner()
: tf_(nullptr), costmap_(nullptr)
{

}

CubicSplinePlanner::~CubicSplinePlanner()
{

}

void CubicSplinePlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
    (void)parent;
    (void)name;
    (void)tf;
    (void)costmap_ros;

    // Create a planner based on the new costmap size
    planner_ = std::make_unique<CubicSpline>();
}

void CubicSplinePlanner::cleanup()
{

}


void CubicSplinePlanner::activate()
{

}


void CubicSplinePlanner::deactivate()
{

}

nav_msgs::msg::Path CubicSplinePlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  (void)start;
  (void)goal;

  nav_msgs::msg::Path path;
  return path;
}

bool CubicSplinePlanner::makePlan(
  const geometry_msgs::msg::Pose & start,
  const geometry_msgs::msg::Pose & goal, double tolerance,
  nav_msgs::msg::Path & plan)
{
  (void)start;
  (void)goal;
  (void)tolerance;
  (void)plan;
  return true;
}

bool CubicSplinePlanner::worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my)
{
  (void)mx;
  (void)my;
  (void)wx;
  (void)wy;
  return true;
}


void CubicSplinePlanner::mapToWorld(double mx, double my, double & wx, double & wy)
{
  (void)mx;
  (void)my;
  (void)wx;
  (void)wy;
}



}  // namespace path_planner
}  // namespace ros2_tutorials