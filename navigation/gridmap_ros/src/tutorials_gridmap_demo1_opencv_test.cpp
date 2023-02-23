#include "gridmap_ros/tutorials_gridmap_demo1_opencv.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto opencv_node = std::make_shared<ros2_tutorials::gridmap_ros::OpencvNode>();
  rclcpp::spin(opencv_node);
  rclcpp::shutdown();
  return 0;
}
