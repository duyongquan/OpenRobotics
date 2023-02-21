#include "topic/tutorials_topic_demo1_initial_env.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto empty_node = std::make_shared<ros2_tutorials::topic::EmptyNode>();
  rclcpp::spin(empty_node);
  rclcpp::shutdown();
  return 0;
}
