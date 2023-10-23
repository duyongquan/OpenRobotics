#ifndef ROS2_COMMON__ROS2_MATH__VEC2D_HPP_
#define ROS2_COMMON__ROS2_MATH__VEC2D_HPP_

#include <cmath>
#include <string>

namespace ros2_common {
namespace ros2_math {

constexpr double kMathEpsilon = 1e-10;

/**
 * @class Vec2d
 *
 * @brief Implements a class of 2-dimensional vectors.
 */
class Vec2d {
 public:
  //! Constructor which takes x- and y-coordinates.
  constexpr Vec2d(const double x, const double y) noexcept : x_(x), y_(y) {}

  //! Constructor returning the zero vector.
  constexpr Vec2d() noexcept : Vec2d(0, 0) {}

  //! Creates a unit-vector with a given angle to the positive x semi-axis
  static Vec2d CreateUnitVec2d(const double angle);

  //! Getter for x component
  double x() const { return x_; }

  //! Getter for y component
  double y() const { return y_; }

  //! Setter for x component
  void set_x(const double x) { x_ = x; }

  //! Setter for y component
  void set_y(const double y) { y_ = y; }

  //! Gets the length of the vector
  double Length() const;

  //! Gets the squared length of the vector
  double LengthSquare() const;

  //! Gets the angle between the vector and the positive x semi-axis
  double Angle() const;

  //! Returns the unit vector that is co-linear with this vector
  void Normalize();

  //! Returns the distance to the given vector
  double DistanceTo(const Vec2d &other) const;

  //! Returns the squared distance to the given vector
  double DistanceSquareTo(const Vec2d &other) const;

  //! Returns the "cross" product between these two Vec2d (non-standard).
  double CrossProd(const Vec2d &other) const;

  //! Returns the inner product between these two Vec2d.
  double InnerProd(const Vec2d &other) const;

  //! rotate the vector by angle.
  Vec2d rotate(const double angle) const;

  //! rotate the vector itself by angle.
  void SelfRotate(const double angle);

  //! Sums two Vec2d
  Vec2d operator+(const Vec2d &other) const;

  //! Subtracts two Vec2d
  Vec2d operator-(const Vec2d &other) const;

  //! Multiplies Vec2d by a scalar
  Vec2d operator*(const double ratio) const;

  //! Divides Vec2d by a scalar
  Vec2d operator/(const double ratio) const;

  //! Sums another Vec2d to the current one
  Vec2d &operator+=(const Vec2d &other);

  //! Subtracts another Vec2d to the current one
  Vec2d &operator-=(const Vec2d &other);

  //! Multiplies this Vec2d by a scalar
  Vec2d &operator*=(const double ratio);

  //! Divides this Vec2d by a scalar
  Vec2d &operator/=(const double ratio);

  //! Compares two Vec2d
  bool operator==(const Vec2d &other) const;

  //! Returns a human-readable string representing this object
  std::string DebugString() const;

 protected:
  double x_ = 0.0;
  double y_ = 0.0;
};

//! Multiplies the given Vec2d by a given scalar
Vec2d operator*(const double ratio, const Vec2d &vec);

}  // ros2_math
}  // ros2_common

#endif  // ROS2_COMMON__ROS2_MATH__VEC2D_HPP_