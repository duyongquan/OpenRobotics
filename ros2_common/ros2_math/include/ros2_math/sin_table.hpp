#ifndef ROS2_COMMON__ROS2_MATH__SIN_TABLE_HPP_
#define ROS2_COMMON__ROS2_MATH__SIN_TABLE_HPP_

namespace ros2_common {
namespace ros2_math {

//! Used by Angle class to speed-up computation of trigonometric functions.
#define SIN_TABLE_SIZE 16385
extern const float SIN_TABLE[SIN_TABLE_SIZE];

}  // ros2_math
}  // ros2_common

#endif  //  ROS2_COMMON__ROS2_MATH__SIN_TABLE_HPP_