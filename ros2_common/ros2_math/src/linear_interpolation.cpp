#include "ros2_math/linear_interpolation.hpp"

#include <cmath>
#include <iostream>

#include "ros2_math/math_utils.hpp"

namespace ros2_common {
namespace ros2_math {

double slerp(const double a0, const double t0, const double a1, const double t1, const double t) 
{
  if (std::abs(t1 - t0) <= kMathEpsilon) {
    std::cout << "input time difference is too small" << std::endl;
    return NormalizeAngle(a0);
  }
  const double a0_n = NormalizeAngle(a0);
  const double a1_n = NormalizeAngle(a1);
  double d = a1_n - a0_n;
  if (d > M_PI) {
    d = d - 2 * M_PI;
  } else if (d < -M_PI) {
    d = d + 2 * M_PI;
  }

  const double r = (t - t0) / (t1 - t0);
  const double a = a0_n + d * r;
  return NormalizeAngle(a);
}

}  // ros2_math
}  // ros2_common