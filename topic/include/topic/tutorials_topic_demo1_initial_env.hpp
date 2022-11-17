
#ifndef ROS2_TUTORIALS__TOPIC_TUTORIALS_TOPIC_DEMO1_INITIAL_ENV_HPP_
#define ROS2_TUTORIALS__TOPIC_TUTORIALS_TOPIC_DEMO1_INITIAL_ENV_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


namespace ros2_tutorials
{
namespace topic
{

class EmptyNode : public rclcpp::Node
{
public:
    EmptyNode();
    ~EmptyNode();
};

}  // namespace topic
}  // namespace ros2_tutorials

#endif  // ROS2_TUTORIALS__TOPIC_TUTORIALS_TOPIC_DEMO1_INITIAL_ENV_HPP_