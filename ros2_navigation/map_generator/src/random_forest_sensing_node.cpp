#include "map_generator/random_forest_sensing.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<map_generator::RandomForestMap>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
