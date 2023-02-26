#include "params/tutorials_params_demo1_yaml_config.hpp"


namespace ros2_tutorials
{
namespace params
{

ParametersParser::ParametersParser()
: Node("minimal_paser")
{
    this->declare_parameter("param_double", 0.0f);
    this->declare_parameter("param_int", 0);
    this->declare_parameter("param_float", 0.);
    this->declare_parameter("param_str", "");

    ReadParams();
}

void ParametersParser::LoadConfigFile()
{
    
}

void ParametersParser::ReadParams()
{
    param_double_ = this->get_parameter("param_double").as_double();
    param_int_ = this->get_parameter("param_int").as_int();
    param_float_ = this->get_parameter("param_float").as_double();
    param_str_ = this->get_parameter("param_str").as_string();

    // Print
    RCLCPP_INFO(this->get_logger(), "double : %lf", param_double_);
    RCLCPP_INFO(this->get_logger(), "int : %d", param_int_);
    RCLCPP_INFO(this->get_logger(), "float : %lf", param_float_);
    RCLCPP_INFO(this->get_logger(), "string : %s", param_str_.c_str());
}

}  // namespace params
}  // namespace ros2_tutorials
