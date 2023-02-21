#include "topic/tutorials_topic_demo3_lambda.hpp"

using namespace std::chrono_literals;

namespace ros2_tutorials
{
namespace topic
{

MinimalPublisherWithLambda::MinimalPublisherWithLambda()
 : Node("lambda_minimal_publisher"), count_(0)
{
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    auto timer_callback = [this]() -> void {
            auto message = std_msgs::msg::String();
            message.data = "Hello, world! " + std::to_string(this->count_++);
            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
            this->publisher_->publish(message);
        };
    timer_ = this->create_wall_timer(500ms, timer_callback);
}


MinimalSubscriberWithLambda::MinimalSubscriberWithLambda()
: Node("labmbda_minimal_subscriber")
{
    subscription_ = this->create_subscription<std_msgs::msg::String>("topic", 10,
        [this](std_msgs::msg::String::UniquePtr msg) {
            RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
        });
}


}  // namespace topic
}  // namespace ros2_tutorials