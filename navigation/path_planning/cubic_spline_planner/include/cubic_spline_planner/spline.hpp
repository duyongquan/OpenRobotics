#ifndef ROS2_TUTORIALS__PATH_PLANNING_SPLINE_HPP_
#define ROS2_TUTORIALS__PATH_PLANNING_SPLINE_HPP_

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

class Spline
{
public:
    Spline();

    Spline(Vec_f x_, Vec_f y_);

    ~Spline();

    float calc(float t);

    float calc_d(float t);

    float calc_dd(float t);

    Vec_f vec_diff(Vec_f input);

    Vec_f cum_sum(Vec_f input);

private:

    Eigen::MatrixXf calc_A();

    Eigen::VectorXf calc_B();

    int bisect(float t, int start, int end);

    Vec_f x;
    Vec_f y;
    int nx;
    Vec_f h;
    Vec_f a;
    Vec_f b;
    Vec_f c;
    Vec_f d;
};

}  // namespace path_planner
}  // namespace ros2_tutorials

#endif  // ROS2_TUTORIALS__PATH_PLANNING_SPLINE_HPP_
