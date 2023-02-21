#ifndef ROS2_TUTORIALS_ACTION__TUTORIALS_ACTION_DEMO1_MEMBER_FUNCTIONS_HPP_
#define ROS2_TUTORIALS_ACTION__TUTORIALS_ACTION_DEMO1_MEMBER_FUNCTIONS_HPP_

#include <chrono>
#include <cinttypes>
#include <functional>
#include <future>
#include <memory>
#include <string>

#include "tutorials_msgs/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace ros2_tutorials
{
namespace action
{

class MinimalActionClient : public rclcpp::Node
{
public:
  using Fibonacci = tutorials_msgs::action::Fibonacci;
  using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;

  explicit MinimalActionClient(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());

  bool is_goal_done() const;

  void send_goal();

private:
  void goal_response_callback(GoalHandleFibonacci::SharedPtr goal_handle);

  void feedback_callback(
    GoalHandleFibonacci::SharedPtr,
    const std::shared_ptr<const Fibonacci::Feedback> feedback);

  void result_callback(const GoalHandleFibonacci::WrappedResult & result);

  rclcpp_action::Client<Fibonacci>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool goal_done_;
};  // class MinimalActionClient

//#####################################################################################

class MinimalActionServer : public rclcpp::Node
{
public:
  using Fibonacci = tutorials_msgs::action::Fibonacci;
  using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

  explicit MinimalActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Fibonacci::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleFibonacci> goal_handle);

  void execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle);
};  // class MinimalActionServer

}  // namespace action
}  // namespace ros2_tutorials

#endif  // ROS2_TUTORIALS_ACTION__TUTORIALS_ACTION_DEMO1_MEMBER_FUNCTIONS_HPP_