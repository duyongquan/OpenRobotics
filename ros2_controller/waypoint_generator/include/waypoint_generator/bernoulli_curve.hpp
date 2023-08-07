#ifndef ROS2_CONTROLLER__BERNOULLI_CURVE_HPP_
#define ROS2_CONTROLLER__BERNOULLI_CURVE_HPP_

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
namespace lemniscate_bernoulli {

/**
 * Adjust this parameter to change the shape of the lemniscate
 *  
 * @param a Default 1.0
 */
nav_msgs::msg::Path CreatePath(double a = 1.0);

}  // namespace lemniscate_bernoulli
}  // namespace waypoint_generator
}  // namespace ros2_controller

#endif  // ROS2_CONTROLLER__BERNOULLI_CURVE_HPP_