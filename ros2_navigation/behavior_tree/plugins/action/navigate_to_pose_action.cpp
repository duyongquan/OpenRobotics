#include <memory>
#include <string>

#include "behavior_tree/plugins/action/navigate_to_pose_action.hpp"

namespace ros2_tutorials
{
namespace behavior_tree
{

NavigateToPoseAction::NavigateToPoseAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_msgs::action::NavigateToPose>(xml_tag_name, action_name, conf)
{
}

void NavigateToPoseAction::on_tick()
{
  if (!getInput("goal", goal_.pose)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "NavigateToPoseAction: goal not provided");
    return;
  }
}

}  // namespace behavior_tree
}  // namespace ros2_tutorials

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<ros2_tutorials::behavior_tree::NavigateToPoseAction>(
        name, "navigate_to_pose", config);
    };

  factory.registerBuilder<ros2_tutorials::behavior_tree::NavigateToPoseAction>(
    "NavigateToPose", builder);
}
