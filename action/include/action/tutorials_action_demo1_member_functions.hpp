

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

}  // namespace action
}  // namespace ros2_tutorials