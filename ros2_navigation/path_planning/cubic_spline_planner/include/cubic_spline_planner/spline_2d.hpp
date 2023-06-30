#ifndef ROS2_TUTORIALS__PATH_PLANNING_SPLINE_2D_HPP_
#define ROS2_TUTORIALS__PATH_PLANNING_SPLINE_2D_HPP_

#include "cubic_spline_planner/spline.hpp"

#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include <vector>
#include <array>
#include <Eigen/Eigen>

namespace ros2_tutorials
{
namespace path_planner
{
/**
 * https://github.com/lsc12318/CppRobotics/blob/master/include/cubic_spline.h
 */
using Vec_f = std::vector<float>;
using Poi_f = std::array<float, 2>;
using Vec_Poi = std::vector<Poi_f>;

class Spline2D
{
public:
    Spline sx;
    Spline sy;
    Vec_f s;

    Spline2D();
    ~Spline2D();

    Spline2D(Vec_f x, Vec_f y);

    Poi_f calc_postion(float s_t);

    float calc_curvature(float s_t);

    float calc_yaw(float s_t);

private:
    Vec_f calc_s(Vec_f x, Vec_f y);
};

}  // namespace path_planner
}  // namespace ros2_tutorials

#endif  // ROS2_TUTORIALS__PATH_PLANNING_SPLINE_2D_HPP_
