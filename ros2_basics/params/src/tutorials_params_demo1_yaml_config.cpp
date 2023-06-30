#include "params/tutorials_params_demo1_yaml_config.hpp"


using rcl_interfaces::msg::ParameterType;

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

    // Setup callback for changes to parameters.
    parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
        this->get_node_base_interface(),
        this->get_node_topics_interface(),
        this->get_node_graph_interface(),
        this->get_node_services_interface());

    parameter_event_sub_ = parameters_client_->on_parameter_event(
        std::bind(&ParametersParser::on_parameter_event_callback, this, std::placeholders::_1));
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

void ParametersParser::on_parameter_event_callback(
    const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
{
    RCLCPP_INFO(this->get_logger(), "Handle parameters events callback function call.");

    for (auto & changed_parameter : event->changed_parameters) {
        const auto & type = changed_parameter.value.type;
        const auto & name = changed_parameter.name;
        const auto & value = changed_parameter.value;

        if (type == ParameterType::PARAMETER_DOUBLE) {
            if (name == "param_double") {
                param_double_ = value.double_value;
            } else if (name == "param_float") {
                param_float_ = value.double_value;
            }
        } else if (type == ParameterType::PARAMETER_STRING) {
            if (name == "param_str") {
                param_str_ = value.string_value;
            }
        } else if (type == ParameterType::PARAMETER_INTEGER) {
            if (name == "param_int") {
                param_int_ = value.integer_value;
            }
        } 
    }

    // Print paramers
    RCLCPP_INFO(this->get_logger(), "double : %lf", param_double_);
    RCLCPP_INFO(this->get_logger(), "int : %d", param_int_);
    RCLCPP_INFO(this->get_logger(), "float : %lf", param_float_);
    RCLCPP_INFO(this->get_logger(), "string : %s", param_str_.c_str());
}

}  // namespace params
}  // namespace ros2_tutorials
