#include "quad_global_planner/rviz_interface.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ros2_quadruped::quad_global_planner::RVizInterface>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
