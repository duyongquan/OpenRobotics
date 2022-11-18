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

}  // namespace topic
}  // namespace ros2_tutorials