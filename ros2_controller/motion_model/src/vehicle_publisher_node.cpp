#include "motion_model/vehicle_publisher.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto vehiclposition_publisher = std::make_shared<ros2_controller::motion_model::VehiclePositionPublisher>();
  rclcpp::spin(vehiclposition_publisher);
  rclcpp::shutdown();
  return 0;
}
