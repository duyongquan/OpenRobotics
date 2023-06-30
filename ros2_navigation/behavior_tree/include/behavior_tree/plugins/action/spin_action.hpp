#ifndef ROS2_TUTORIALS_BEHAVIOR_TREE__PLUGINS__ACTION__SPIN_ACTION_HPP_
#define ROS2_TUTORIALS_BEHAVIOR_TREE__PLUGINS__ACTION__SPIN_ACTION_HPP_

#include <string>

#include "behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/spin.hpp"

namespace ros2_tutorials
{
namespace behavior_tree
{


/**
 * @brief A nav2_behavior_tree::BtActionNode class that wraps nav2_msgs::action::Spin
 */
class SpinAction : public BtActionNode<nav2_msgs::action::Spin>
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::SpinAction
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  SpinAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Function to perform some user-defined operation on tick
   */
  void on_tick() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<double>("spin_dist", 1.57, "Spin distance"),
        BT::InputPort<double>("time_allowance", 10.0, "Allowed time for spinning")
      });
  }
};

}  // namespace behavior_tree
}  // namespace ros2_tutorials

#endif  // ROS2_TUTORIALS_BEHAVIOR_TREE__PLUGINS__ACTION__SPIN_ACTION_HPP_
