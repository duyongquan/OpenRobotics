#include "topic/tutorials_topic_demo5_member_function_with_topic_statistics.hpp"

using namespace std::chrono_literals;

namespace ros2_tutorials
{
namespace topic
{

MinimalSubscriberWithTopicStatistics::MinimalSubscriberWithTopicStatistics()
: Node("minimal_subscriber_with_topic_statistics")
{
    // manually enable topic statistics via options
    auto options = rclcpp::SubscriptionOptions();
    options.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;

    // configure the collection window and publish period (default 1s)
    options.topic_stats_options.publish_period = std::chrono::seconds(10);

    // configure the topic name (default '/statistics')
    // options.topic_stats_options.publish_topic = "/topic_statistics"

    auto callback = [this](std_msgs::msg::String::SharedPtr msg) {
        this->HandleTopicCallback(msg);
    };

    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "topic", 10, callback, options);
}

void MinimalSubscriberWithTopicStatistics::HandleTopicCallback(
    const std_msgs::msg::String::SharedPtr msg) const
{
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
}

}  // namespace topic
}  // namespace ros2_tutorials