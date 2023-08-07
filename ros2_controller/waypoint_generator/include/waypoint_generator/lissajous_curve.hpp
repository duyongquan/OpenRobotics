#ifndef ROS2_CONTROLLER__LISSAJOUS_CURVE_HPP_
#define ROS2_CONTROLLER__LISSAJOUS_CURVE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"

#include <chrono>
#include <cinttypes>
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <cmath>

namespace ros2_controller {
namespace waypoint_generator {
namespace lissajous_curve {

/**
 * Adjust this parameter to change the shape of the lemniscate
 *  
 * @param a Default 1.0
 */
nav_msgs::msg::Path CreatePath();

}  // namespace lissajous_curve
}  // namespace waypoint_generator
}  // namespace ros2_controller

#endif  // ROS2_CONTROLLER__LISSAJOUS_CURVE_HPP_