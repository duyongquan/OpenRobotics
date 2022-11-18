#ifndef ROS2_TUTORIALS__TOPIC_TUTORIALS_TOPIC_DEMO3_LAMBDA_HPP_
#define ROS2_TUTORIALS__TOPIC_TUTORIALS_TOPIC_DEMO3_LAMBDA_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


namespace ros2_tutorials
{
namespace topic
{

/* This example creates a subclass of Node and uses a fancy C++11 lambda
 * function to shorten the callback syntax, at the expense of making the
 * code somewhat more difficult to understand at first glance. */

class MinimalPublisherWithLambda : public rclcpp::Node
{
public:
  MinimalPublisherWithLambda();

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

}  // namespace topic
}  // namespace ros2_tutorials

#endif  // ROS2_TUTORIALS__TOPIC_TUTORIALS_TOPIC_DEMO3_LAMBDA_HPP_