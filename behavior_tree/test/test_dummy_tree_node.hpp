#ifndef TEST_DUMMY_TREE_NODE_HPP_
#define TEST_DUMMY_TREE_NODE_HPP_

#include <behaviortree_cpp_v3/basic_types.h>
#include <behaviortree_cpp_v3/action_node.h>

namespace behavior_tree
{

/**
 * @brief A Dummy TreeNode to be used as a child for testing nodes
 * Returns the current status on tick without any execution logic
 */
class DummyNode : public BT::ActionNodeBase
{
public:
  DummyNode()
  : BT::ActionNodeBase("dummy", {})
  {
  }

  void changeStatus(BT::NodeStatus status)
  {
    setStatus(status);
  }

  BT::NodeStatus executeTick() override
  {
    return tick();
  }

  BT::NodeStatus tick() override
  {
    return status();
  }

  void halt() override
  {
  }
};

}  // namespace behavior_tree

#endif  // TEST_DUMMY_TREE_NODE_HPP_
