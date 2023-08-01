#include "quad_global_planner/global_body_planner.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ros2_quadruped::quad_global_planner::GlobalBodyPlanner>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
