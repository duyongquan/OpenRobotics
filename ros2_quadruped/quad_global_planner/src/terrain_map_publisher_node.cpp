
#include "quad_global_planner/terrain_map_publisher.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ros2_quadruped::quad_global_planner::TerrainMapPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
