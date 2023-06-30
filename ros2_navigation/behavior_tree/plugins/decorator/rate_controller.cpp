#include <chrono>
#include <string>

#include "behavior_tree/plugins/decorator/rate_controller.hpp"

namespace ros2_tutorials
{
namespace behavior_tree
{

RateController::RateController(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf),
  first_time_(false)
{
  double hz = 1.0;
  getInput("hz", hz);
  period_ = 1.0 / hz;
}

BT::NodeStatus RateController::tick()
{
  if (status() == BT::NodeStatus::IDLE) {
    // Reset the starting point since we're starting a new iteration of
    // the rate controller (moving from IDLE to RUNNING)
    start_ = std::chrono::high_resolution_clock::now();
    first_time_ = true;
  }

  setStatus(BT::NodeStatus::RUNNING);

  // Determine how long its been since we've started this iteration
  auto now = std::chrono::high_resolution_clock::now();
  auto elapsed = now - start_;

  // Now, get that in seconds
  typedef std::chrono::duration<float> float_seconds;
  auto seconds = std::chrono::duration_cast<float_seconds>(elapsed);

  // The child gets ticked the first time through and any time the period has
  // expired. In addition, once the child begins to run, it is ticked each time
  // 'til completion
  if (first_time_ || (child_node_->status() == BT::NodeStatus::RUNNING) ||
    seconds.count() >= period_)
  {
    first_time_ = false;
    const BT::NodeStatus child_state = child_node_->executeTick();

    switch (child_state) {
      case BT::NodeStatus::RUNNING:
        return BT::NodeStatus::RUNNING;

      case BT::NodeStatus::SUCCESS:
        start_ = std::chrono::high_resolution_clock::now();  // Reset the timer
        return BT::NodeStatus::SUCCESS;

      case BT::NodeStatus::FAILURE:
      default:
        return BT::NodeStatus::FAILURE;
    }
  }

  return status();
}

}  // namespace behavior_tree
}  // namespace ros2_tutorials

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<ros2_tutorials::behavior_tree::RateController>("RateController");
}
