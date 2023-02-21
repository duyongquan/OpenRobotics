#ifndef ROS2_TUTORIALS__TUTORIALS_TIMER_DEMO1_INITIAL_ENV_HPP_
#define ROS2_TUTORIALS__TUTORIALS_TIMER_DEMO1_INITIAL_ENV_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

namespace ros2_tutorials
{
namespace timer
{

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */
class MinimalTimer : public rclcpp::Node
{
public:
  MinimalTimer();

private:
  void HandleTimerCallback();

  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace timer
}  // namespace ros2_tutorials

#endif  // ROS2_TUTORIALS__TUTORIALS_TIMER_DEMO1_INITIAL_ENV_HPP_