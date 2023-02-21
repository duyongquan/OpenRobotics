<center>  <font size=10 color='green'> ros2_tutorials —— action使用 </font></center>

<center> <font color=red size=8> Part I 客户端(Client & Server) </font> </center>

# 1 tutorial 1(member_functions_test)

## 1.1 功能介绍

> 客户端fibonacci请求，服务器fibonacci生成fibonacci数据
>
> fibonacci： 1， 2， 3， 5， 8，13，  ...

## 1.2 代码

**头文件tutorials_action_demo1_member_functions.hpp**

```c++
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

```

**tutorials_action_demo1_member_functions.cpp**

```cpp
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
```

**测试文件tutorials_action_demo1_member_functions_client_test.cpp**

```cpp
#include "action/tutorials_action_demo1_member_functions.hpp"

// int main(int argc, char * argv[])
// {
//   rclcpp::init(argc, argv);

//   rclcpp::executors::MultiThreadedExecutor executor;
//   auto action_server = std::make_shared<ros2_tutorials::action::MinimalActionServer>();
//   auto action_client = std::make_shared<ros2_tutorials::action::MinimalActionClient>();

//   while (!action_client->is_goal_done()) {
//     executor.add_node(action_server);
//     executor.add_node(action_client);
//   }

//   executor.spin();
//   rclcpp::shutdown();
//   return 0;
// }

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto action_server = std::make_shared<ros2_tutorials::action::MinimalActionClient>();

  rclcpp::spin(action_server);

  rclcpp::shutdown();
  return 0;
}
```

**测试文件tutorials_action_demo1_member_functions_server_test.cpp**

```cpp
#include "action/tutorials_action_demo1_member_functions.hpp"

// int main(int argc, char * argv[])
// {
//   rclcpp::init(argc, argv);

//   rclcpp::executors::MultiThreadedExecutor executor;
//   auto action_server = std::make_shared<ros2_tutorials::action::MinimalActionServer>();
//   auto action_client = std::make_shared<ros2_tutorials::action::MinimalActionClient>();

//   while (!action_client->is_goal_done()) {
//     executor.add_node(action_server);
//     executor.add_node(action_client);
//   }

//   executor.spin();
//   rclcpp::shutdown();
//   return 0;
// }

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto action_server = std::make_shared<ros2_tutorials::action::MinimalActionServer>();

  rclcpp::spin(action_server);

  rclcpp::shutdown();
  return 0;
}

```



## 1.3 编译

```perl
colcon build  --packages-up-to action
```

## 1.4 运行

source环境变量

```shell
source install/setup.zsh
```

```shell
ros2 launch action tutorials.action.demo1_member_function_test.launch.py
```

## 1.5 运行结果

![topic_tutorial.topic.initial_env_test](./images/tutorials_action_demo1_member_functions.png)



