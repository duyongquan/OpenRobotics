#include "cubic_spline_planner/cubic_spline_planner.hpp"

namespace ros2_tutorials
{
namespace path_planner
{

CubicSplinePlanner::CubicSplinePlanner()
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
    
}


}  // namespace path_planner
}  // namespace ros2_tutorials