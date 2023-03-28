#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "quad_gazebo/contact_state_publisher.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ros2_quadruped::quad_gazebo::ContactStatePublisher>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}
