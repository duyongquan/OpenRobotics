#ifndef ROS2_TUTORIALS__TUTORIALS_TOPIC_DEMO5_MEMBER_FUNCTION_WITH_TOPIC_STATISTICS_HPP_
#define ROS2_TUTORIALS__TUTORIALS_TOPIC_DEMO5_MEMBER_FUNCTION_WITH_TOPIC_STATISTICS_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


namespace ros2_tutorials
{
namespace topic
{

class MinimalSubscriberWithTopicStatistics : public rclcpp::Node
{
public:
  MinimalSubscriberWithTopicStatistics();

private:
  void HandleTopicCallback(const std_msgs::msg::String::SharedPtr msg) const;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

}  // namespace topic
}  // namespace ros2_tutorials

#endif  // ROS2_TUTORIALS__TUTORIALS_TOPIC_DEMO5_MEMBER_FUNCTION_WITH_TOPIC_STATISTICS_HPP_