#include <string>
#include "behavior_tree/plugins/control/recovery_node.hpp"

namespace ros2_tutorials
{
namespace behavior_tree
{

RecoveryNode::RecoveryNode(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ControlNode::ControlNode(name, conf),
  current_child_idx_(0),
  number_of_retries_(1),
  retry_count_(0)
{
  getInput("number_of_retries", number_of_retries_);
}

BT::NodeStatus RecoveryNode::tick()
{
  const unsigned children_count = children_nodes_.size();

  if (children_count != 2) {
    throw BT::BehaviorTreeException("Recovery Node '" + name() + "' must only have 2 children.");
  }

  setStatus(BT::NodeStatus::RUNNING);

  while (current_child_idx_ < children_count && retry_count_ <= number_of_retries_) {
    TreeNode * child_node = children_nodes_[current_child_idx_];
    const BT::NodeStatus child_status = child_node->executeTick();

    if (current_child_idx_ == 0) {
      switch (child_status) {
        case BT::NodeStatus::SUCCESS:
          {
            // reset node and return success when first child returns success
            halt();
            return BT::NodeStatus::SUCCESS;
          }

        case BT::NodeStatus::FAILURE:
          {
            if (retry_count_ < number_of_retries_) {
              // halt first child and tick second child in next iteration
              ControlNode::haltChild(0);
              current_child_idx_++;
              break;
            } else {
              // reset node and return failure when max retries has been exceeded
              halt();
              return BT::NodeStatus::FAILURE;
            }
          }

        case BT::NodeStatus::RUNNING:
          {
            return BT::NodeStatus::RUNNING;
          }

        default:
          {
            throw BT::LogicError("A child node must never return IDLE");
          }
      }  // end switch

    } else if (current_child_idx_ == 1) {
      switch (child_status) {
        case BT::NodeStatus::SUCCESS:
          {
            // halt second child, increment recovery count, and tick first child in next iteration
            ControlNode::haltChild(1);
            retry_count_++;
            current_child_idx_--;
          }
          break;

        case BT::NodeStatus::FAILURE:
          {
            // reset node and return failure if second child fails
            halt();
            return BT::NodeStatus::FAILURE;
          }

        case BT::NodeStatus::RUNNING:
          {
            return BT::NodeStatus::RUNNING;
          }

        default:
          {
            throw BT::LogicError("A child node must never return IDLE");
          }
      }  // end switch
    }
  }  // end while loop

  // reset node and return failure
  halt();
  return BT::NodeStatus::FAILURE;
}

void RecoveryNode::halt()
{
  ControlNode::halt();
  retry_count_ = 0;
  current_child_idx_ = 0;
}

}  // namespace nav2_behavior_tree
}  // namespace ros2_tutorials

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<ros2_tutorials::behavior_tree::RecoveryNode>("RecoveryNode");
}
