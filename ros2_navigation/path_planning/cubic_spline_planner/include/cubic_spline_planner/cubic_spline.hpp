#ifndef ROS2_TUTORIALS__PATH_PLANNING_CUBIC_SPLINE_HPP_
#define ROS2_TUTORIALS__PATH_PLANNING_CUBIC_SPLINE_HPP_

#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

namespace ros2_tutorials
{
namespace path_planner
{

class CubicSpline
{
public:
  /**
   * @brief  Constructs the planner
   * @param nx The x size of the map
   * @param ny The y size of the map
   */
  CubicSpline();

  ~CubicSpline();
};

}  // namespace path_planner
}  // namespace ros2_tutorials

#endif  // ROS2_TUTORIALS__PATH_PLANNING_CUBIC_SPLINE_HPP_
