#ifndef ROS2_COMMON__ROS2_MATH__QUATERNION_HPP_
#define ROS2_COMMON__ROS2_MATH__QUATERNION_HPP_

/**
 * @file
 * @brief Contains a number of helper functions related to quaternions.
 * The reader should refer to euler_angles_zxy.h for clarifications.
 * However, although the vehicle frame defined therein might change,
 * the definition of the heading of the car, used below, is permanently fixed:
 * 0 at East, pi/2 at North, -pi/2 at South.
 */

#include <cmath>

#include "Eigen/Dense"
#include "Eigen/Geometry"

#include "ros2_math/euler_angles_zxy.hpp"
#include "ros2_math/math_utils.hpp"

namespace ros2_common {
namespace ros2_math {

/*
 * @brief Returns heading (in radians) in [-PI, PI), with 0 being East.
 * Note that x/y/z is East/North/Up.
 *
 * @param qw Quaternion w-coordinate
 * @param qx Quaternion x-coordinate
 * @param qy Quaternion y-coordinate
 * @param qz Quaternion z-coordinate
 *
 * @return Heading encoded by given quaternion
 */
inline double QuaternionToHeading(const double qw, const double qx,
                                  const double qy, const double qz) {
  EulerAnglesZXYd euler_angles(qw, qx, qy, qz);
  // euler_angles.yaw() is zero when the car is pointing North, but
  // the heading is zero when the car is pointing East.
  return NormalizeAngle(euler_angles.yaw() + M_PI_2);
}

/*
 * @brief Returns heading (in radians) in [-PI, PI), with 0 being East.
 * Note that x/y/z is East/North/Up.
 *
 * @param q Eigen::Quaternion
 *
 * @return Heading encoded by given quaternion
 */
template <typename T>
inline T QuaternionToHeading(const Eigen::Quaternion<T> &q) {
  return static_cast<T>(QuaternionToHeading(q.w(), q.x(), q.y(), q.z()));
}

/*
 * @brief Returns a quaternion encoding a rotation with zero roll, zero pitch,
 * and the specified heading/yaw.
 * Note that heading is zero at East and yaw is zero at North.
 * Satisfies QuaternionToHeading(HeadingToQuaternion(h)) = h.
 *
 * @param heading The heading to encode in the rotation
 *
 * @return Quaternion encoding rotation by given heading
 */
template <typename T>
inline Eigen::Quaternion<T> HeadingToQuaternion(T heading) {
  // Note that heading is zero at East and yaw is zero at North.
  EulerAnglesZXY<T> euler_angles(heading - M_PI_2);
  return euler_angles.ToQuaternion();
}

/*
 * @brief Applies the rotation defined by a quaternion to a given vector.
 * Note that x/y/z is East/North/Up.
 *
 * @param orientation Quaternion
 * @param original Vector (in East-North-Up frame)
 *
 * @return Rotated vector
 */
inline Eigen::Vector3d QuaternionRotate(const Quaternion &orientation,
                                        const Eigen::Vector3d &original) {
  Eigen::Quaternion<double> quaternion(orientation.qw(), orientation.qx(),
                                       orientation.qy(), orientation.qz());
  return static_cast<Eigen::Vector3d>(quaternion.toRotationMatrix() * original);
}

inline Eigen::Vector3d InverseQuaternionRotate(const Quaternion &orientation,
                                               const Eigen::Vector3d &rotated) {
  Eigen::Quaternion<double> quaternion(orientation.qw(), orientation.qx(),
                                       orientation.qy(), orientation.qz());
  return static_cast<Eigen::Vector3d>(quaternion.toRotationMatrix().inverse() *
                                      rotated);
}

}  // ros2_math
}  // ros2_common

#endif  // ROS2_COMMON__ROS2_MATH__QUATERNION_HPP_