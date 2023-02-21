#ifndef ROS2_TUTORIALS_BEHAVIOR_TREE__PLUGINS__CONDITION__GOAL_REACHED_CONDITION_HPP_
#define ROS2_TUTORIALS_BEHAVIOR_TREE__PLUGINS__CONDITION__GOAL_REACHED_CONDITION_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/condition_node.h"
#include "tf2_ros/buffer.h"

namespace ros2_tutorials {
namespace behavior_tree
{

/**
 * @brief A BT::ConditionNode that returns SUCCESS when a specified goal
 * is reached and FAILURE otherwise
 */
class GoalReachedCondition : public BT::ConditionNode
{
public:
  /**
   * @brief A constructor for behavior_tree::GoalReachedCondition
   * @param condition_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  GoalReachedCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  GoalReachedCondition() = delete;

  /**
   * @brief A destructor for behavior_tree::GoalReachedCondition
   */
  ~GoalReachedCondition() override;

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Function to read parameters and initialize class variables
   */
  void initialize();

  /**
   * @brief Checks if the current robot pose lies within a given distance from the goal
   * @return bool true when goal is reached, false otherwise
   */
  bool isGoalReached();

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Destination"),
      BT::InputPort<std::string>("global_frame", std::string("map"), "Global frame"),
      BT::InputPort<std::string>("robot_base_frame", std::string("base_link"), "Robot base frame")
    };
  }

protected:
  /**
   * @brief Cleanup function
   */
  void cleanup()
  {}

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;

  bool initialized_;
  double goal_reached_tol_;
  std::string global_frame_;
  std::string robot_base_frame_;
  double transform_tolerance_;
};

}  // namespace behavior_tree
}  // namespace ros2_tutorials

#endif  // ROS2_TUTORIALS_BEHAVIOR_TREE__PLUGINS__CONDITION__GOAL_REACHED_CONDITION_HPP_
