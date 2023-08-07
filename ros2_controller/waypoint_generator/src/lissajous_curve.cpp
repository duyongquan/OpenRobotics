#include "waypoint_generator/lissajous_curve.hpp"

namespace ros2_controller {
namespace waypoint_generator {
namespace lissajous_curve {

/**
 * Adjust this parameter to change the shape of the lemniscate
 *  
 * @param a Default 1.0
 */
nav_msgs::msg::Path CreatePath()
{
    nav_msgs::msg::Path path;
    path.header.stamp =  rclcpp::Clock().now();
    path.header.frame_id = "odom";

    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.z = 0.0;

    // for (double angle = 0.0; angle <= 2 * M_PI; angle += 0.01)
    // {
    //     double x = a * std::sqrt(2 * std::cos(2 * angle)) * std::cos(angle);
    //     double y = a * std::sqrt(2 * std::cos(2 * angle)) * std::sin(angle);

    //     if (!std::isnan(x) || !std::isnan(y)) {
    //         pose.pose.position.x = x;
    //         pose.pose.position.y = y;
    //         path.poses.push_back(pose);
    //     } 
    // }

    double A = 8.0;  // Amplitude in x-direction
    double B = 6.0;  // Amplitude in y-direction
    double a = 1.0;  // Frequency in x-direction
    double b = 2.0;  // Frequency in y-direction
    double delta = 1.57;  // Phase difference

    for (double t = 0.0; t <= 2 * M_PI; t += 0.01)
    {
        pose.pose.position.x = A * sin(a * t + delta);
        pose.pose.position.y = B * sin(b * t);
        pose.pose.position.z = 0.0;
        path.poses.push_back(pose);
    }

    return path;
}

}  // namespace lissajous_curve
}  // namespace waypoint_generator
}  // namespace ros2_controller
