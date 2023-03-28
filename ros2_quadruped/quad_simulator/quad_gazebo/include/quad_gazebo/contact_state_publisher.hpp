#ifndef ROS2_QUADRUPED__QUAD_SIMULATOR_QUAD_GAZEBO_CONTACT_STATE_PUBLISHER_HPP_
#define ROS2_QUADRUPED__QUAD_SIMULATOR_QUAD_GAZEBO_CONTACT_STATE_PUBLISHER_HPP_

#include "rclcpp/rclcpp.hpp"

namespace ros2_quadruped {
namespace quad_gazebo {

class ContactStatePublisher : public rclcpp::Node
{
public:
    ContactStatePublisher();
    ~ContactStatePublisher();

private:
};

}  // namespace quad_gazebo
}  // namespace ros2_quadruped

#endif  // ROS2_QUADRUPED__QUAD_SIMULATOR_QUAD_GAZEBO_CONTACT_STATE_PUBLISHER_HPP_