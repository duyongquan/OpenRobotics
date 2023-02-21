#ifndef ROS2_TUTORIALS__TOPIC_TUTORIALS_TOPIC_DEMO2_MEMBER_FUNCTION_HPP_
#define ROS2_TUTORIALS__TOPIC_TUTORIALS_TOPIC_DEMO2_MEMBER_FUNCTION_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace ros2_tutorials
{
namespace topic
{

/**
 * This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. 
 */
class MinimalPublisher : public rclcpp::Node
{
public:
    MinimalPublisher();
    ~MinimalPublisher();

private:
    void HandleTimerCallback();
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber();

private:
  void HandleTopicCallback(const std_msgs::msg::String::SharedPtr msg) const;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

}  // namespace topic
}  // namespace ros2_tutorials

#endif  // ROS2_TUTORIALS__TOPIC_TUTORIALS_TOPIC_DEMO2_MEMBER_FUNCTION_HPP_