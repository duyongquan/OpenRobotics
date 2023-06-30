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

    /**
     * @brief Callback executed when a paramter change is detected
     * @param event ParameterEvent message
     */
    void on_parameter_event_callback(const rcl_interfaces::msg::ParameterEvent::SharedPtr event);

    // Subscription for parameter change
    rclcpp::AsyncParametersClient::SharedPtr parameters_client_;
    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;

    double param_double_;
    int param_int_;
    float param_float_;
    std::string param_str_;
};

}  // namespace params
}  // namespace ros2_tutorials

#endif  // ROS2_TUTORIALS__TUTORIALS_PARAMS_DEMO1_YAML_CONFIG_HPP_