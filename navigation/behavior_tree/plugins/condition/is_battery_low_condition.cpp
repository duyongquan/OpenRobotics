#include <string>

#include "behavior_tree/plugins/condition/is_battery_low_condition.hpp"

namespace ros2_tutorials {
namespace behavior_tree
{

IsBatteryLowCondition::IsBatteryLowCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  battery_topic_("/battery_status"),
  min_battery_(0.0),
  is_voltage_(false),
  is_battery_low_(false)
{
  getInput("min_battery", min_battery_);
  getInput("battery_topic", battery_topic_);
  getInput("is_voltage", is_voltage_);
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  battery_sub_ = node_->create_subscription<sensor_msgs::msg::BatteryState>(
    battery_topic_,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&IsBatteryLowCondition::batteryCallback, this, std::placeholders::_1),
    sub_option);
}

BT::NodeStatus IsBatteryLowCondition::tick()
{
  callback_group_executor_.spin_some();
  if (is_battery_low_) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

void IsBatteryLowCondition::batteryCallback(sensor_msgs::msg::BatteryState::SharedPtr msg)
{
  if (is_voltage_) {
    is_battery_low_ = msg->voltage <= min_battery_;
  } else {
    is_battery_low_ = msg->percentage <= min_battery_;
  }
}

}  // namespace behavior_tree
}  // namespace ros2_tutorials

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<ros2_tutorials::behavior_tree::IsBatteryLowCondition>("IsBatteryLow");
}
