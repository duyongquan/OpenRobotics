#include "timer/tutorials_timer_demo1_initial_env.hpp"

namespace ros2_tutorials
{
namespace timer
{

MinimalTimer::MinimalTimer()
: Node("minimal_timer")
{
    timer_ = create_wall_timer(
        500ms, std::bind(&MinimalTimer::HandleTimerCallback, this));
}

void MinimalTimer::HandleTimerCallback()
{
    RCLCPP_INFO(this->get_logger(), "Hello, world!");
}

}  // namespace timer
}  // namespace ros2_tutorials