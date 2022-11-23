
#ifndef ROS2_TUTORIALS__TUTORIALS_RVIZ_DEMO1_INITIAL_ENV_HPP_
#define ROS2_TUTORIALS__TUTORIALS_RVIZ_DEMO1_INITIAL_ENV_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/path.hpp"

namespace ros2_tutorials
{
namespace rviz
{

class EmptyNode : public rclcpp::Node
{
public:
    EmptyNode();
    ~EmptyNode();
};


class ShowCircleTrajectory : public rclcpp::Node
{
public:
    ShowCircleTrajectory();
    ~ShowCircleTrajectory();

private:
    void PublishPath();
    void HandleTimerCallback();

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace rviz
}  // namespace ros2_tutorials

#endif  // ROS2_TUTORIALS__TUTORIALS_RVIZ_DEMO1_INITIAL_ENV_HPP_