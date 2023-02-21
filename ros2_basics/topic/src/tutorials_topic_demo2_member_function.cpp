#include "topic/tutorials_topic_demo2_member_function.hpp"

using namespace std::chrono_literals;

namespace ros2_tutorials
{
namespace topic
{

MinimalPublisher::MinimalPublisher()
: Node("member_minimal_publisher"), count_(0)
{
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisher::HandleTimerCallback, this));
}

MinimalPublisher::~MinimalPublisher()
{
}

void MinimalPublisher::HandleTimerCallback()
{
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
}

//####################################################################################
MinimalSubscriber::MinimalSubscriber()
: Node("member_minimal_subscriber")
{
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "topic", 10, std::bind(&MinimalSubscriber::HandleTopicCallback, this, std::placeholders::_1));
}

void MinimalSubscriber::HandleTopicCallback(const std_msgs::msg::String::SharedPtr msg) const
{
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
}

}  // namespace topic
}  // namespace ros2_tutorials