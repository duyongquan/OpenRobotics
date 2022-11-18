#include "topic/tutorials_topic_demo2_member_function.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto publisher_node = std::make_shared<ros2_tutorials::topic::MinimalPublisher>();
  rclcpp::spin(publisher_node);
  rclcpp::shutdown();
  return 0;
}