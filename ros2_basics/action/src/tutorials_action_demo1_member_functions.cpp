#include "action/tutorials_action_demo1_member_functions.hpp"

namespace ros2_tutorials
{
namespace action
{

MinimalActionClient::MinimalActionClient(const rclcpp::NodeOptions & node_options)
: Node("minimal_action_client", node_options),
  goal_done_(false)
{
    this->client_ptr_ = rclcpp_action::create_client<Fibonacci>(
        this->get_node_base_interface(),
        this->get_node_graph_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(),
        "fibonacci");

    this->timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&MinimalActionClient::send_goal, this));
}

bool MinimalActionClient::is_goal_done() const
{
    return this->goal_done_;
}

void MinimalActionClient::send_goal()
{
    using namespace std::placeholders;

    this->timer_->cancel();

    this->goal_done_ = false;

    if (!this->client_ptr_) {
      RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
    }

    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      this->goal_done_ = true;
      return;
    }

    auto goal_msg = Fibonacci::Goal();
    goal_msg.order = 10;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&MinimalActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&MinimalActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&MinimalActionClient::result_callback, this, _1);
    auto goal_handle_future = this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
}

void MinimalActionClient::goal_response_callback(GoalHandleFibonacci::SharedPtr goal_handle)
{
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
}

void MinimalActionClient::feedback_callback(
    GoalHandleFibonacci::SharedPtr,
    const std::shared_ptr<const Fibonacci::Feedback> feedback)
{
    RCLCPP_INFO(
        this->get_logger(),
        "Next number in sequence received: %" PRId32, feedback->sequence.back());
}

void MinimalActionClient::result_callback(const GoalHandleFibonacci::WrappedResult & result)
{
    this->goal_done_ = true;
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
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

    RCLCPP_INFO(this->get_logger(), "Result received");
    for (auto number : result.result->sequence) {
        RCLCPP_INFO(this->get_logger(), "%" PRId32, number);
    }
}

//#####################################################################################

MinimalActionServer::MinimalActionServer(const rclcpp::NodeOptions & options)
: Node("minimal_action_server", options)
{
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<Fibonacci>(
        this->get_node_base_interface(),
        this->get_node_clock_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(),
        "fibonacci",
        std::bind(&MinimalActionServer::handle_goal, this, _1, _2),
        std::bind(&MinimalActionServer::handle_cancel, this, _1),
        std::bind(&MinimalActionServer::handle_accepted, this, _1));
}


rclcpp_action::GoalResponse MinimalActionServer::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Fibonacci::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);
    (void)uuid;
    // Let's reject sequences that are over 9000
    if (goal->order > 9000) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MinimalActionServer::handle_cancel(
    const std::shared_ptr<GoalHandleFibonacci> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void MinimalActionServer::execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Fibonacci::Feedback>();
    auto & sequence = feedback->sequence;
    sequence.push_back(0);
    sequence.push_back(1);
    auto result = std::make_shared<Fibonacci::Result>();

    for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->sequence = sequence;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal Canceled");
        return;
      }
      // Update sequence
      sequence.push_back(sequence[i] + sequence[i - 1]);
      // Publish feedback
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish Feedback");

      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->sequence = sequence;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal Succeeded");
    }
}

void MinimalActionServer::handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
{
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&MinimalActionServer::execute, this, _1), goal_handle}.detach();
}

}  // namespace action
}  // namespace ros2_tutorials