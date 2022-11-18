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

}  // namespace topic
}  // namespace ros2_tutorials