#ifndef ROS2_TUTORIALS__TUTORIALS_PARAMS_DEMO1_YAML_CONFIG_HPP_
#define ROS2_TUTORIALS__TUTORIALS_PARAMS_DEMO1_YAML_CONFIG_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


namespace ros2_tutorials
{
namespace params
{

class ParametersParser : public rclcpp::Node
{
public:
    ParametersParser();

private:
    void LoadConfigFile();
    void ReadParams();

    double param_double_;
    int param_int_;
    float param_float_;
    std::string param_str_;
};

}  // namespace params
}  // namespace ros2_tutorials

#endif  // ROS2_TUTORIALS__TUTORIALS_PARAMS_DEMO1_YAML_CONFIG_HPP_