#ifndef ROS2_QUADRUPED__QUAD_NMPC_CONTROLLER__QUAD_NLP_HPP_
#define ROS2_QUADRUPED__QUAD_NMPC_CONTROLLER__QUAD_NLP_HPP_

#include "rclcpp/rclcpp.hpp"


namespace ros2_quadruped {
namespace quad_nmpc_controller {

enum SystemID 
{
    SPIRIT,
    A1,
    SIMPLE_TO_SIMPLE,
    SIMPLE_TO_COMPLEX,
    COMPLEX_TO_COMPLEX,
    COMPLEX_TO_SIMPLE
};

enum FunctionID 
{
    FUNC,
    JAC, 
    HESS 
};

}  // namespace quad_nmpc_controller
}  // namespace ros2_quadruped

#endif  // ROS2_QUADRUPED__QUAD_NMPC_CONTROLLER__QUAD_NLP_HPP_