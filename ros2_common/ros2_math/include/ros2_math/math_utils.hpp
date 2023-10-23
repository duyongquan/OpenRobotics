#ifndef ROS2_COMMON__ROS2_MATH__MATH_UTILS_HPP_
#define ROS2_COMMON__ROS2_MATH__MATH_UTILS_HPP_

#include <cmath>
#include <limits>
#include <type_traits>
#include <utility>

#include "Eigen/Dense"

#include "ros2_math/vec2d.hpp"

namespace ros2_common {
namespace ros2_math {

double Sqr(const double x);

/**
 * @brief Cross product between two 2-D vectors from the common start point,
 *        and end at two other points.
 * @param start_point The common start point of two vectors in 2-D.
 * @param end_point_1 The end point of the first vector.
 * @param end_point_2 The end point of the second vector.
 *
 * @return The cross product result.
 */
double CrossProd(const Vec2d &start_point, const Vec2d &end_point_1,
                 const Vec2d &end_point_2);

/**
 * @brief Inner product between two 2-D vectors from the common start point,
 *        and end at two other points.
 * @param start_point The common start point of two vectors in 2-D.
 * @param end_point_1 The end point of the first vector.
 * @param end_point_2 The end point of the second vector.
 *
 * @return The inner product result.
 */
double InnerProd(const Vec2d &start_point, const Vec2d &end_point_1,
                 const Vec2d &end_point_2);

/**
 * @brief Cross product between two vectors.
 *        One vector is formed by 1st and 2nd parameters of the function.
 *        The other vector is formed by 3rd and 4th parameters of the function.
 * @param x0 The x coordinate of the first vector.
 * @param y0 The y coordinate of the first vector.
 * @param x1 The x coordinate of the second vector.
 * @param y1 The y coordinate of the second vector.
 *
 * @return The cross product result.
 */
double CrossProd(const double x0, const double y0, const double x1,
                 const double y1);

/**
 * @brief Inner product between two vectors.
 *        One vector is formed by 1st and 2nd parameters of the function.
 *        The other vector is formed by 3rd and 4th parameters of the function.
 * @param x0 The x coordinate of the first vector.
 * @param y0 The y coordinate of the first vector.
 * @param x1 The x coordinate of the second vector.
 * @param y1 The y coordinate of the second vector.
 *
 * @return The inner product result.
 */
double InnerProd(const double x0, const double y0, const double x1,
                 const double y1);

/**
 * @brief Wrap angle to [0, 2 * PI).
 * @param angle the original value of the angle.
 * @return The wrapped value of the angle.
 */
double WrapAngle(const double angle);

/**
 * @brief Normalize angle to [-PI, PI).
 * @param angle the original value of the angle.
 * @return The normalized value of the angle.
 */
double NormalizeAngle(const double angle);

/**
 * @brief Calculate the difference between angle from and to
 * @param from the start angle
 * @param from the end angle
 * @return The difference between from and to. The range is between [-PI, PI).
 */
double AngleDiff(const double from, const double to);

/**
 * @brief Get a random integer between two integer values by a random seed.
 * @param s The lower bound of the random integer.
 * @param t The upper bound of the random integer.
 * @param random_seed The random seed.
 * @return A random integer between s and t based on the input random_seed.
 */
int RandomInt(const int s, const int t, unsigned int rand_seed = 1);

/**
 * @brief Get a random double between two integer values by a random seed.
 * @param s The lower bound of the random double.
 * @param t The upper bound of the random double.
 * @param random_seed The random seed.
 * @return A random double between s and t based on the input random_seed.
 */
double RandomDouble(const double s, const double t, unsigned int rand_seed = 1);

/**
 * @brief Compute squared value.
 * @param value The target value to get its squared value.
 * @return Squared value of the input value.
 */
template <typename T>
inline T Square(const T value) {
  return value * value;
}

/**
 * @brief Clamp a value between two bounds.
 *        If the value goes beyond the bounds, return one of the bounds,
 *        otherwise, return the original value.
 * @param value The original value to be clamped.
 * @param bound1 One bound to clamp the value.
 * @param bound2 The other bound to clamp the value.
 * @return The clamped value.
 */
template <typename T>
T Clamp(const T value, T bound1, T bound2) {
  if (bound1 > bound2) {
    std::swap(bound1, bound2);
  }

  if (value < bound1) {
    return bound1;
  } else if (value > bound2) {
    return bound2;
  }
  return value;
}

// Gaussian
double Gaussian(const double u, const double std, const double x);

inline double Sigmoid(const double x) { return 1.0 / (1.0 + std::exp(-x)); }

// Rotate a 2d vector counter-clockwise by theta
Eigen::Vector2d RotateVector2d(const Eigen::Vector2d &v_in, const double theta);

inline std::pair<double, double> RFUToFLU(const double x, const double y) {
  return std::make_pair(y, -x);
}

inline std::pair<double, double> FLUToRFU(const double x, const double y) {
  return std::make_pair(-y, x);
}

inline void L2Norm(int feat_dim, float *feat_data) {
  if (feat_dim == 0) {
    return;
  }
  // feature normalization
  float l2norm = 0.0f;
  for (int i = 0; i < feat_dim; ++i) {
    l2norm += feat_data[i] * feat_data[i];
  }
  if (l2norm == 0) {
    float val = 1.f / std::sqrt(static_cast<float>(feat_dim));
    for (int i = 0; i < feat_dim; ++i) {
      feat_data[i] = val;
    }
  } else {
    l2norm = std::sqrt(l2norm);
    for (int i = 0; i < feat_dim; ++i) {
      feat_data[i] /= l2norm;
    }
  }
}

// Cartesian coordinates to Polar coordinates
std::pair<double, double> Cartesian2Polar(double x, double y);

template <class T>
typename std::enable_if<!std::numeric_limits<T>::is_integer, bool>::type
almost_equal(T x, T y, int ulp) {
  // the machine epsilon has to be scaled to the magnitude of the values used
  // and multiplied by the desired precision in ULPs (units in the last place)
  // unless the result is subnormal
  return std::fabs(x - y) <=
             std::numeric_limits<T>::epsilon() * std::fabs(x + y) * ulp ||
         std::fabs(x - y) < std::numeric_limits<T>::min();
}

}  // ros2_math
}  // ros2_common

#endif  //  ROS2_COMMON__ROS2_MATH__MATH_UTILS_HPP_