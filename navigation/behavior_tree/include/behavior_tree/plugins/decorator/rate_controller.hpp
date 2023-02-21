#ifndef ROS2_TUTORIALS__PLUGINS__DECORATOR__RATE_CONTROLLER_HPP_
#define ROS2_TUTORIALS__PLUGINS__DECORATOR__RATE_CONTROLLER_HPP_

#include <chrono>
#include <string>

#include "behaviortree_cpp_v3/decorator_node.h"

namespace ros2_tutorials
{
namespace behavior_tree
{

/**
 * @brief A BT::DecoratorNode that ticks its child at a specified rate
 */
class RateController : public BT::DecoratorNode
{
public:
  RateController(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("hz", 10.0, "Rate")
    };
  }

private:
  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  std::chrono::time_point<std::chrono::high_resolution_clock> start_;
  double period_;
  bool first_time_;
};

}  // namespace behavior_tree
}  // namespace ros2_tutorials

#endif  // ROS2_TUTORIALS__PLUGINS__DECORATOR__RATE_CONTROLLER_HPP_