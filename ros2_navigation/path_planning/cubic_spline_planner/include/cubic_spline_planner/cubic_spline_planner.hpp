#ifndef ROS2_TUTORIALS__PATH_PLANNING_CUBIC_SPLINE_PLANNER_HPP_
#define ROS2_TUTORIALS__PATH_PLANNING_CUBIC_SPLINE_PLANNER_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/geometry_utils.hpp"

#include "cubic_spline_planner/cubic_spline.hpp"

using namespace std::chrono_literals;

namespace ros2_tutorials
{
namespace path_planner
{

class CubicSplinePlanner : public nav2_core::GlobalPlanner
{
public:
/**
   * @brief constructor
   */
  CubicSplinePlanner();

  /**
   * @brief destructor
   */
  ~CubicSplinePlanner();

  /**
   * @brief Configuring plugin
   * @param parent Lifecycle node pointer
   * @param name Name of plugin map
   * @param tf Shared ptr of TF2 buffer
   * @param costmap_ros Costmap2DROS object
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief Cleanup lifecycle node
   */
  void cleanup() override;

  /**
   * @brief Activate lifecycle node
   */
  void activate() override;

  /**
   * @brief Deactivate lifecycle node
   */
  void deactivate() override;


  /**
   * @brief Creating a plan from start and goal poses
   * @param start Start pose
   * @param goal Goal pose
   * @return nav_msgs::Path of the generated path
   */
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;
protected:
  /**
   * @brief Compute a plan given start and goal poses, provided in global world frame.
   * @param start Start pose
   * @param goal Goal pose
   * @param tolerance Relaxation constraint in x and y
   * @param plan Path to be computed
   * @return true if can find the path
   */
  bool makePlan(
    const geometry_msgs::msg::Pose & start,
    const geometry_msgs::msg::Pose & goal, double tolerance,
    nav_msgs::msg::Path & plan);

  /**
   * @brief Transform a point from world to map frame
   * @param wx double of world X coordinate
   * @param wy double of world Y coordinate
   * @param mx int of map X coordinate
   * @param my int of map Y coordinate
   * @return true if can transform
   */
  bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my);

  /**
   * @brief Transform a point from map to world frame
   * @param mx double of map X coordinate
   * @param my double of map Y coordinate
   * @param wx double of world X coordinate
   * @param wy double of world Y coordinate
   */
  void mapToWorld(double mx, double my, double & wx, double & wy);

  // Planner based on ROS1 NavFn algorithm
  std::unique_ptr<CubicSpline> planner_;

  // TF buffer
  std::shared_ptr<tf2_ros::Buffer> tf_;

  // Clock
  rclcpp::Clock::SharedPtr clock_;

  // Logger
  rclcpp::Logger logger_{rclcpp::get_logger("CubicSplinePlanner")};

  // Global Costmap
  nav2_costmap_2d::Costmap2D * costmap_;

  // The global frame of the costmap
  std::string global_frame_, name_;

  // If the goal is obstructed, the tolerance specifies how many meters the planner
  // can relax the constraint in x and y before failing
  double tolerance_;

  // Whether to use the astar planner or default dijkstras
  bool use_astar_;

  // Subscription for parameter change
  rclcpp::AsyncParametersClient::SharedPtr parameters_client_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;

  /**
   * @brief Callback executed when a paramter change is detected
   * @param event ParameterEvent message
   */
  void on_parameter_event_callback(const rcl_interfaces::msg::ParameterEvent::SharedPtr event);
};

}  // namespace path_planner
}  // namespace ros2_tutorials

#endif  // ROS2_TUTORIALS__PATH_PLANNING_CUBIC_SPLINE_PLANNER_HPP_