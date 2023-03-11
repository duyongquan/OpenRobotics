#ifndef ROS2_QUADRUPED__QUAD_SIMULATOR_QUAD_UTILS_TRAJECTORY_PUBLISHER_HPP_
#define ROS2_QUADRUPED__QUAD_SIMULATOR_QUAD_UTILS_TRAJECTORY_PUBLISHER_HPP_

#include "rclcpp/rclcpp.hpp"

namespace ros2_quadruped {
namespace quad_utils {

class TrajectoryPublisher : public rclcpp::Node
{
public:
    TrajectoryPublisher();
    ~TrajectoryPublisher();

private:
};

}  // namespace quad_utils
}  // namespace ros2_quadruped

#endif  // ROS2_QUADRUPED__QUAD_SIMULATOR_QUAD_UTILS_TRAJECTORY_PUBLISHER_HPP_