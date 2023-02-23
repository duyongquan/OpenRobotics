#ifndef ROS2_TUTORIALS_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_STUCK_CONDITION_HPP_
#define ROS2_TUTORIALS_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_STUCK_CONDITION_HPP_

#include <string>
#include <atomic>
#include <deque>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/condition_node.h"
#include "nav_msgs/msg/odometry.hpp"

namespace ros2_tutorials
{
namespace behavior_tree
{

/**
 * @brief A BT::ConditionNode that tracks robot odometry and returns SUCCESS
 * if robot is stuck somewhere and FAILURE otherwise
 */
class IsStuckCondition : public BT::ConditionNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::IsStuckCondition
   * @param condition_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  IsStuckCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  IsStuckCondition() = delete;

  /**
   * @brief A destructor for nav2_behavior_tree::IsStuckCondition
   */
  ~IsStuckCondition() override;

  /**
   * @brief Callback function for odom topic
   * @param msg Shared pointer to nav_msgs::msg::Odometry::SharedPtr message
   */
  void onOdomReceived(const typename nav_msgs::msg::Odometry::SharedPtr msg);

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Function to log status when robot is stuck/free
   */
  void logStuck(const std::string & msg) const;

  /**
   * @brief Function to approximate acceleration from the odom history
   */
  void updateStates();

  /**
   * @brief Detect if robot bumped into something by checking for abnormal deceleration
   * @return bool true if robot is stuck, false otherwise
   */
  bool isStuck();

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts() {return {};}

private:
  // The node that will be used for any ROS operations
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  std::thread callback_group_executor_thread;

  std::atomic<bool> is_stuck_;

  // Listen to odometry
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  // Store history of odometry measurements
  std::deque<nav_msgs::msg::Odometry> odom_history_;
  std::deque<nav_msgs::msg::Odometry>::size_type odom_history_size_;

  // Calculated states
  double current_accel_;

  // Robot specific paramters
  double brake_accel_limit_;
};

}  // namespace behavior_tree
}  // namespace ros2_tutorials

#endif  // ROS2_TUTORIALS_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_STUCK_CONDITION_HPP_
