#include "quad_teleop/teleop_twist_joy.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ros2_quadruped::quad_teleop::TeleopTwistJoy>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
