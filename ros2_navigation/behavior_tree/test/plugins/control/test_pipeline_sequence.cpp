
#include <gtest/gtest.h>
#include <memory>

#include "../../test_behavior_tree_fixture.hpp"
#include "../../test_dummy_tree_node.hpp"
#include "behavior_tree/plugins/control/pipeline_sequence.hpp"

class PipelineSequenceTestFixture : public behavior_tree::BehaviorTreeTestFixture
{
public:
  void SetUp() override
  {
    bt_node_ = std::make_shared<ros2_tutorials::behavior_tree::PipelineSequence>(
      "pipeline_sequence", *config_);
    first_child_ = std::make_shared<behavior_tree::DummyNode>();
    second_child_ = std::make_shared<behavior_tree::DummyNode>();
    third_child_ = std::make_shared<behavior_tree::DummyNode>();
    bt_node_->addChild(first_child_.get());
    bt_node_->addChild(second_child_.get());
    bt_node_->addChild(third_child_.get());
  }

  void TearDown() override
  {
    first_child_.reset();
    second_child_.reset();
    third_child_.reset();
    bt_node_.reset();
  }

protected:
  static std::shared_ptr<ros2_tutorials::behavior_tree::PipelineSequence> bt_node_;
  static std::shared_ptr<behavior_tree::DummyNode> first_child_;
  static std::shared_ptr<behavior_tree::DummyNode> second_child_;
  static std::shared_ptr<behavior_tree::DummyNode> third_child_;
};

std::shared_ptr<ros2_tutorials::behavior_tree::PipelineSequence>
PipelineSequenceTestFixture::bt_node_ = nullptr;
std::shared_ptr<behavior_tree::DummyNode>
PipelineSequenceTestFixture::first_child_ = nullptr;
std::shared_ptr<behavior_tree::DummyNode>
PipelineSequenceTestFixture::second_child_ = nullptr;
std::shared_ptr<behavior_tree::DummyNode>
PipelineSequenceTestFixture::third_child_ = nullptr;

TEST_F(PipelineSequenceTestFixture, test_failure_on_idle_child)
{
  first_child_->changeStatus(BT::NodeStatus::IDLE);
  EXPECT_THROW(bt_node_->executeTick(), std::runtime_error);
}

TEST_F(PipelineSequenceTestFixture, test_failure)
{
  first_child_->changeStatus(BT::NodeStatus::FAILURE);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(first_child_->status(), BT::NodeStatus::IDLE);
  EXPECT_EQ(second_child_->status(), BT::NodeStatus::IDLE);
  EXPECT_EQ(third_child_->status(), BT::NodeStatus::IDLE);

  first_child_->changeStatus(BT::NodeStatus::SUCCESS);
  second_child_->changeStatus(BT::NodeStatus::FAILURE);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(first_child_->status(), BT::NodeStatus::IDLE);
  EXPECT_EQ(second_child_->status(), BT::NodeStatus::IDLE);
  EXPECT_EQ(third_child_->status(), BT::NodeStatus::IDLE);

  first_child_->changeStatus(BT::NodeStatus::SUCCESS);
  second_child_->changeStatus(BT::NodeStatus::SUCCESS);
  third_child_->changeStatus(BT::NodeStatus::FAILURE);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(first_child_->status(), BT::NodeStatus::IDLE);
  EXPECT_EQ(second_child_->status(), BT::NodeStatus::IDLE);
  EXPECT_EQ(third_child_->status(), BT::NodeStatus::IDLE);

  first_child_->changeStatus(BT::NodeStatus::SUCCESS);
  second_child_->changeStatus(BT::NodeStatus::SUCCESS);
  third_child_->changeStatus(BT::NodeStatus::RUNNING);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::RUNNING);
  first_child_->changeStatus(BT::NodeStatus::FAILURE);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(first_child_->status(), BT::NodeStatus::IDLE);
  EXPECT_EQ(second_child_->status(), BT::NodeStatus::IDLE);
  EXPECT_EQ(third_child_->status(), BT::NodeStatus::IDLE);
}

TEST_F(PipelineSequenceTestFixture, test_behavior)
{
  first_child_->changeStatus(BT::NodeStatus::RUNNING);
  second_child_->changeStatus(BT::NodeStatus::IDLE);
  third_child_->changeStatus(BT::NodeStatus::IDLE);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::RUNNING);

  first_child_->changeStatus(BT::NodeStatus::SUCCESS);
  second_child_->changeStatus(BT::NodeStatus::RUNNING);
  third_child_->changeStatus(BT::NodeStatus::IDLE);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::RUNNING);

  first_child_->changeStatus(BT::NodeStatus::RUNNING);
  second_child_->changeStatus(BT::NodeStatus::SUCCESS);
  third_child_->changeStatus(BT::NodeStatus::RUNNING);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::RUNNING);

  first_child_->changeStatus(BT::NodeStatus::RUNNING);
  second_child_->changeStatus(BT::NodeStatus::SUCCESS);
  third_child_->changeStatus(BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(first_child_->status(), BT::NodeStatus::IDLE);
  EXPECT_EQ(second_child_->status(), BT::NodeStatus::IDLE);
  EXPECT_EQ(third_child_->status(), BT::NodeStatus::IDLE);
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