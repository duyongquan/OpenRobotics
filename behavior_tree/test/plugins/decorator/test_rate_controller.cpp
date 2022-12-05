#include <gtest/gtest.h>
#include <chrono>
#include <memory>
#include <set>

#include "../../test_behavior_tree_fixture.hpp"
#include "behavior_tree/plugins/decorator/rate_controller.hpp"

using namespace std::chrono;  // NOLINT
using namespace std::chrono_literals;  // NOLINT

class RateControllerTestFixture : public behavior_tree::BehaviorTreeTestFixture
{
public:
  void SetUp()
  {
    bt_node_ = std::make_shared<ros2_tutorials::behavior_tree::RateController>(
      "rate_controller", *config_);
    dummy_node_ = std::make_shared<behavior_tree::DummyNode>();
    bt_node_->setChild(dummy_node_.get());
  }

  void TearDown()
  {
    dummy_node_.reset();
    bt_node_.reset();
  }

protected:
  static std::shared_ptr<ros2_tutorials::behavior_tree::RateController> bt_node_;
  static std::shared_ptr<behavior_tree::DummyNode> dummy_node_;
};

std::shared_ptr<ros2_tutorials::behavior_tree::RateController>
RateControllerTestFixture::bt_node_ = nullptr;
std::shared_ptr<behavior_tree::DummyNode>
RateControllerTestFixture::dummy_node_ = nullptr;

TEST_F(RateControllerTestFixture, test_behavior)
{
  EXPECT_EQ(bt_node_->status(), BT::NodeStatus::IDLE);

  dummy_node_->changeStatus(BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(dummy_node_->status(), BT::NodeStatus::IDLE);

  for (int i = 0; i < 10; ++i) {
    dummy_node_->changeStatus(BT::NodeStatus::SUCCESS);
    std::this_thread::sleep_for(500ms);
    if (i % 2) {
      EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
      EXPECT_EQ(dummy_node_->status(), BT::NodeStatus::IDLE);
    } else {
      EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::RUNNING);
    }
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // initialize ROS
  rclcpp::init(argc, argv);

  bool all_successful = RUN_ALL_TESTS();

  // shutdown ROS
  rclcpp::shutdown();

  return all_successful;
}