#include "topic/tutorials_topic_demo3_lambda.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto empty_node = std::make_shared<ros2_tutorials::topic::MinimalPublisherWithLambda>();
  rclcpp::spin(empty_node);
  rclcpp::shutdown();
  return 0;
}