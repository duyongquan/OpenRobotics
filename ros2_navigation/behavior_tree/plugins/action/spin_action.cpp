#include <string>
#include <memory>

#include "behavior_tree/plugins/action/spin_action.hpp"

namespace ros2_tutorials
{
namespace behavior_tree
{

SpinAction::SpinAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_msgs::action::Spin>(xml_tag_name, action_name, conf)
{
  double dist;
  getInput("spin_dist", dist);
  double time_allowance;
  getInput("time_allowance", time_allowance);
  goal_.target_yaw = dist;
//   goal_.time_allowance = rclcpp::Duration::from_seconds(time_allowance);
}

void SpinAction::on_tick()
{
  increment_recovery_count();
}

}  // namespace behavior_tree
}  // namespace ros2_tutorials

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<ros2_tutorials::behavior_tree::SpinAction>(name, "spin", config);
    };

  factory.registerBuilder<ros2_tutorials::behavior_tree::SpinAction>("Spin", builder);
}