#ifndef ROS2_COMMON__ROS2_MATH__LINEAR_INTERPOLATION_HPP_
#define ROS2_COMMON__ROS2_MATH__LINEAR_INTERPOLATION_HPP_

#include <cmath>
#include <iostream>

namespace ros2_common {
namespace ros2_math {

/**
 * @brief Linear interpolation between two points of type T.
 * @param x0 The coordinate of the first point.
 * @param t0 The interpolation parameter of the first point.
 * @param x1 The coordinate of the second point.
 * @param t1 The interpolation parameter of the second point.
 * @param t The interpolation parameter for interpolation.
 * @param x The coordinate of the interpolated point.
 * @return Interpolated point.
 */
template <typename T>
T lerp(const T &x0, const double t0, const T &x1, const double t1, const double t) 
{
  if (std::abs(t1 - t0) <= 1.0e-6) {
    std::cerr << "input time difference is too small" << std::endl;
    return x0;
  }
  const double r = (t - t0) / (t1 - t0);
  const T x = x0 + r * (x1 - x0);
  return x;
}

/**
 * @brief Spherical linear interpolation between two angles.
 *        The two angles are within range [-M_PI, M_PI).
 * @param a0 The value of the first angle.
 * @param t0 The interpolation parameter of the first angle.
 * @param a1 The value of the second angle.
 * @param t1 The interpolation parameter of the second angle.
 * @param t The interpolation parameter for interpolation.
 * @param a The value of the spherically interpolated angle.
 * @return Interpolated angle.
 */
double slerp(const double a0, const double t0, const double a1, const double t1, const double t);

}  // ros2_math
}  // ros2_common

#endif  // ROS2_COMMON__ROS2_MATH__LINEAR_INTERPOLATION_HPP_
