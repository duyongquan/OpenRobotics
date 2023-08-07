#include "waypoint_generator/bernoulli_curve.hpp"

namespace ros2_controller {
namespace waypoint_generator {
namespace lemniscate_bernoulli {

/**
 * Adjust this parameter to change the shape of the lemniscate
 *  
 * @param a Default 1.0
 */
nav_msgs::msg::Path CreatePath(double a)
{
    nav_msgs::msg::Path path;
    path.header.stamp =  rclcpp::Clock().now();
    path.header.frame_id = "odom";

    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.z = 0.0;

    for (double angle = 0.0; angle <= 2 * M_PI; angle += 0.01)
    {
        double x = a * std::sqrt(2 * std::cos(2 * angle)) * std::cos(angle);
        double y = a * std::sqrt(2 * std::cos(2 * angle)) * std::sin(angle);

        if (!std::isnan(x) || !std::isnan(y)) {
            pose.pose.position.x = x;
            pose.pose.position.y = y;
            path.poses.push_back(pose);
        } 
    }

    return path;
}

}  // namespace lemniscate_bernoulli
}  // namespace waypoint_generator
}  // namespace ros2_controller
