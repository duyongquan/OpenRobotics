#include "timer/tutorials_timer_demo1_initial_env.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ros2_tutorials::timer::MinimalTimer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
