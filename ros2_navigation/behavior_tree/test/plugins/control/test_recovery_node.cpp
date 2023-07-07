#include <gtest/gtest.h>
#include <memory>

#include "../../test_behavior_tree_fixture.hpp"
#include "../../test_dummy_tree_node.hpp"
#include "behavior_tree/plugins/control/recovery_node.hpp"

// Changes status to SUCCESS after a specified number of failures
class RecoveryDummy : public behavior_tree::DummyNode
{
public:
  BT::NodeStatus tick() override
  {
    if (ticks_ == num_success_) {
      setStatus(BT::NodeStatus::SUCCESS);
    } else if (ticks_ == num_failure_) {
      setStatus(BT::NodeStatus::FAILURE);
    }
    ticks_++;
    return status();
  }

  void returnSuccessOn(int tick)
  {
    num_success_ = tick;
    ticks_ = 0;
  }

  void returnFailureOn(int tick)
  {
    num_failure_ = tick;
    ticks_ = 0;
  }

private:
  int ticks_{0};
  int num_success_{-1};
  int num_failure_{-1};
};

class RecoveryNodeTestFixture : public behavior_tree::BehaviorTreeTestFixture
{
public:
  void SetUp() override
  {
    bt_node_ = std::make_shared<ros2_tutorials::behavior_tree::RecoveryNode>(
      "recovery_node", *config_);
    first_child_ = std::make_shared<RecoveryDummy>();
    second_child_ = std::make_shared<RecoveryDummy>();
    bt_node_->addChild(first_child_.get());
    bt_node_->addChild(second_child_.get());
  }

  void TearDown() override
  {
    first_child_.reset();
    second_child_.reset();
    bt_node_.reset();
  }

protected:
  static std::shared_ptr<ros2_tutorials::behavior_tree::RecoveryNode> bt_node_;
  static std::shared_ptr<RecoveryDummy> first_child_;
  static std::shared_ptr<RecoveryDummy> second_child_;
};

std::shared_ptr<ros2_tutorials::behavior_tree::RecoveryNode> RecoveryNodeTestFixture::bt_node_ = nullptr;
std::shared_ptr<RecoveryDummy> RecoveryNodeTestFixture::first_child_ = nullptr;
std::shared_ptr<RecoveryDummy> RecoveryNodeTestFixture::second_child_ = nullptr;

TEST_F(RecoveryNodeTestFixture, test_only_two_children)
{
  behavior_tree::DummyNode dummy;
  bt_node_->addChild(&dummy);
  EXPECT_THROW(bt_node_->executeTick(), BT::BehaviorTreeException);
}

TEST_F(RecoveryNodeTestFixture, test_running)
{
  first_child_->changeStatus(BT::NodeStatus::RUNNING);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::RUNNING);
  first_child_->changeStatus(BT::NodeStatus::FAILURE);
  second_child_->changeStatus(BT::NodeStatus::RUNNING);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::RUNNING);
}

TEST_F(RecoveryNodeTestFixture, test_failure_on_idle_child)
{
  first_child_->changeStatus(BT::NodeStatus::IDLE);
  EXPECT_THROW(bt_node_->executeTick(), BT::LogicError);
  first_child_->changeStatus(BT::NodeStatus::FAILURE);
  second_child_->changeStatus(BT::NodeStatus::IDLE);
  EXPECT_THROW(bt_node_->executeTick(), BT::LogicError);
}

TEST_F(RecoveryNodeTestFixture, test_success_one_retry)
{
  // first child returns success right away
  first_child_->changeStatus(BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);

  // first child fails, second child succeeds, then first child succeeds (one retry)
  first_child_->returnSuccessOn(1);
  first_child_->changeStatus(BT::NodeStatus::FAILURE);
  second_child_->changeStatus(BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bt_node_->status(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(first_child_->status(), BT::NodeStatus::IDLE);
  EXPECT_EQ(second_child_->status(), BT::NodeStatus::IDLE);
}

TEST_F(RecoveryNodeTestFixture, test_failure_one_retry)
{
  // first child fails, second child fails
  first_child_->changeStatus(BT::NodeStatus::FAILURE);
  second_child_->changeStatus(BT::NodeStatus::FAILURE);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(bt_node_->status(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(first_child_->status(), BT::NodeStatus::IDLE);
  EXPECT_EQ(second_child_->status(), BT::NodeStatus::IDLE);

  // first child fails, second child succeeds, then first child fails (one retry)
  first_child_->returnFailureOn(1);
  first_child_->changeStatus(BT::NodeStatus::FAILURE);
  second_child_->changeStatus(BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(bt_node_->status(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(first_child_->status(), BT::NodeStatus::IDLE);
  EXPECT_EQ(second_child_->status(), BT::NodeStatus::IDLE);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // initialize ROS
  rclcpp::init(argc, argv);

  int all_successful = RUN_ALL_TESTS();

  // shutdown ROS
  rclcpp::shutdown();

  return all_successful;
}