#include "params/tutorials_params_demo1_yaml_config.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ros2_tutorials::params::ParametersParser>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
