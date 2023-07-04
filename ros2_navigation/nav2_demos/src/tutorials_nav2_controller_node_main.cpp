
#include <chrono>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "nav2_demos/tutorials_nav2_utils.hpp"

namespace ros2_tutorials
{
namespace nav2
{
namespace
{



class ControllerNode : public rclcpp::Node
{
public:
    using FollowPath = nav2_msgs::action::FollowPath;
    using GoalHandleFollowPath = rclcpp_action::ClientGoalHandle<FollowPath>;

    ControllerNode() : Node("path_controller")
    {
        
        controller_ = std::make_shared<NavControllerTester>();
        global_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "plan", rclcpp::SystemDefaultsQoS(),
                std::bind(&ControllerNode::HandleFollowPathCallback, this, std::placeholders::_1));

        action_client_ = rclcpp_action::create_client<FollowPath>(
            this->get_node_base_interface(),
            this->get_node_graph_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(),
            "follow_path");

        controller_->Activate();
        initialize_finished_ = true;

        RCLCPP_INFO(this->get_logger(), "Initialize controller OK.");
    }

    ~ControllerNode() {}

private:

    void HandleFollowPathCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        if (!initialize_finished_) {
            return;
        }

        if (msg == nullptr || msg->poses.empty()) {
            return;
        }

        if (following_path_) {
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Handle FollowPathCallback.");

        controller_->SetPlannerPath(*msg);

        // StartRunning();

        following_path_ = true;
    }

    void StartRunning()
    {
        action_client_->wait_for_action_server();
        auto send_goal_options = rclcpp_action::Client<FollowPath>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&ControllerNode::HandleGoalResponseCallback, this, std::placeholders::_1);
        send_goal_options.feedback_callback =
            std::bind(&ControllerNode::HandleFeedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback =
            std::bind(&ControllerNode::HandleResultCallback, this, std::placeholders::_1);

        RCLCPP_INFO(this->get_logger(), "Start running controller.");

        // Send goal
        auto goal = FollowPath::Goal();
        auto goal_handle_future = action_client_->async_send_goal(goal, send_goal_options);
    }

    void HandleGoalResponseCallback(GoalHandleFollowPath::SharedPtr goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void HandleFeedbackCallback(
        GoalHandleFollowPath::SharedPtr,
        const std::shared_ptr<const FollowPath::Feedback> feedback)
    {
    }

    void HandleResultCallback(const GoalHandleFollowPath::WrappedResult & result)
    {
        switch (result.code) 
        {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Goal was successed");
                break;
            
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                return;
    
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                return;

            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                return;
        }

        following_path_ = false;
    }

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_{nullptr};
    std::shared_ptr<NavControllerTester> controller_ {nullptr};
    rclcpp_action::Client<FollowPath>::SharedPtr action_client_{nullptr};

    bool following_path_{false};
    bool initialize_finished_{false};
};

}  // namespace
}  // namespace nav2
}  // namespace ros2_tutorials



int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto controller_node = std::make_shared<ros2_tutorials::nav2::ControllerNode>();
  rclcpp::spin(controller_node);
  rclcpp::shutdown();
  return 0;
}
