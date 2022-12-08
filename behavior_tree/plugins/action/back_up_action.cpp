#include <string>
#include <memory>

#include "behavior_tree/plugins/action/back_up_action.hpp"

namespace ros2_tutorials
{
namespace behavior_tree
{

BackUpAction::BackUpAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_msgs::action::BackUp>(xml_tag_name, action_name, conf)
{
  double dist;
  getInput("backup_dist", dist);
  double speed;
  getInput("backup_speed", speed);
  double time_allowance;
  getInput("time_allowance", time_allowance);

  // Populate the input message
  goal_.target.x = dist;
  goal_.target.y = 0.0;
  goal_.target.z = 0.0;
  goal_.speed = speed;
//   goal_.time_allowance = rclcpp::Duration::from_seconds(time_allowance);
}

void BackUpAction::on_tick()
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
      return std::make_unique<ros2_tutorials::behavior_tree::BackUpAction>(
        name, "backup", config);
    };

  factory.registerBuilder<ros2_tutorials::behavior_tree::BackUpAction>("BackUp", builder);
}
