#ifndef ROS2_QUADRUPED__QUAD_TELEOP__TELEOP_TWIST_JOY_HPP_
#define ROS2_QUADRUPED__QUAD_TELEOP__TELEOP_TWIST_JOY_HPP_


#include "rclcpp/rclcpp.hpp"

namespace ros2_quadruped {
namespace quad_teleop {

/**
 * Class implementing a basic Joy -> Twist translation.
 */
class TeleopTwistJoy : public rclcpp::Node
{
public:
    TeleopTwistJoy();

private:
    struct Impl;
    Impl* pimpl_;
};

}  // namespace quad_teleop
}  // namespace ros2_quadruped

#endif   // ROS2_QUADRUPED__QUAD_TELEOP__TELEOP_TWIST_JOY_HPP_
