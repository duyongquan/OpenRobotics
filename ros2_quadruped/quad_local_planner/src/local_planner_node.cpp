#include "quad_local_planner/local_planner.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ros2_quadruped::quad_local_planner::LocalPlanner>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
