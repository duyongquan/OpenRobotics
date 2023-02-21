#include "rviz/tutorials_rviz_demo1_initial_env.hpp"

#include <cmath>

using namespace std::chrono_literals;

namespace ros2_tutorials
{
namespace rviz
{

EmptyNode::EmptyNode()
: Node("empty")
{
    RCLCPP_INFO(get_logger(), "Creating empty node.");
}

EmptyNode::~EmptyNode()
{
}

ShowCircleTrajectory::ShowCircleTrajectory()
: Node("TrajectoryCircle")
{
    path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("circle_trajectory", 10);
    timer_ = this->create_wall_timer(1s, std::bind(&ShowCircleTrajectory::HandleTimerCallback, this));
}

ShowCircleTrajectory::~ShowCircleTrajectory()
{
}

void ShowCircleTrajectory::HandleTimerCallback()
{
    PublishPath();
}

void ShowCircleTrajectory::PublishPath()
{
    const double pi_2 = 6.28;
    auto path = nav_msgs::msg::Path();

    path.header.stamp =  this->get_clock()->now();
    path.header.frame_id = "map";

    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.z = 0.0;

    // circle
    for (std::size_t i = 0; i < 1000 ; i++) {

        double theta =  (i * pi_2 )/ 1000;
        double radius = 5.0f;

        double x = radius * std::cos(theta);
        double y = radius * std::sin(theta);
        pose.pose.position.x = x;
        pose.pose.position.y = y;

        // std::cout << "i = " << i << std::endl;
        path.poses.push_back(pose);
    }

    path_publisher_->publish(path);
}

}  // namespace rviz
}  // namespace ros2_tutorials