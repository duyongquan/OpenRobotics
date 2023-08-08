#include "waypoint_generator/trajectory_generation.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2/impl/utils.h>

namespace ros2_controller {
namespace waypoint_generator {


TrajectoryGenerator::TrajectoryGenerator()
{

}

nav_msgs::msg::Path TrajectoryGenerator::CreatePath(const TrajectoryType& type)
{
    switch (type)
    {
    case TrajectoryType::Circle:
        return CreateCirclePath();
        break;

     case TrajectoryType::Epitrochoid:
        return CreateEpitrochoidPath();
        break;

     case TrajectoryType::Square:
        return CreateSquarePath();
        break;

     case TrajectoryType::Infinite:
        return CreateInfinitePath();
        break;
    
    default:
        break;
    }

    return CreateCirclePath();
}


nav_msgs::msg::Path TrajectoryGenerator::CreateCirclePath()
{
    // std::cout << "Create circle path" << std::endl;

    auto path = nav_msgs::msg::Path(); 
    path.header.stamp =  rclcpp::Clock().now();
    path.header.frame_id = "odom";

    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.z = 0.0;

    double radius = 5.0; // Radius of the circle
    double frequency = 0.05; // Frequency of publishing points
    double angular_velocity = 2.0 * M_PI * frequency;

    for (double theta = 0.0; theta <= 2.0 * M_PI; theta += angular_velocity * 0.05)
    {
        pose.pose.position.x = radius * std::cos(theta);
        pose.pose.position.y = radius * std::sin(theta);
        double next_theta = theta + angular_velocity * 0.05;

        double dy = -radius * std::cos(next_theta) - pose.pose.position.y;
        double dx =  radius * std::sin(next_theta) - (pose.pose.position.x + 1e-5);
        double yaw =  std::atan2(dy, dx);
        tf2::Quaternion quaternion;
        quaternion.setRPY(0, 0, yaw);

        pose.pose.orientation.x = quaternion.x();
        pose.pose.orientation.y = quaternion.y();
        pose.pose.orientation.z = quaternion.z();
        pose.pose.orientation.w = quaternion.w();
        // pose.pose.orientation.w = 1.0; // Assuming orientation is not important for your use case
        path.poses.push_back(pose);
    }

    return path;
}

nav_msgs::msg::Path TrajectoryGenerator::CreateEpitrochoidPath()
{
    // std::cout << "Create epitrochoid path" << std::endl;
    auto path = nav_msgs::msg::Path(); 
    path.header.stamp =  rclcpp::Clock().now();
    path.header.frame_id = "odom";

    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.z = 0.0;

    double radius = 5.0; // Radius of the circle
    double frequency = 0.05; // Frequency of publishing points
    double angular_velocity = 2.0 * M_PI * frequency;

    int R = 5;
    int r = 1;
    int d = 3;
    int scale_factor = 1;
    

    for (double theta = 0.0; theta <= 2.0 * M_PI; theta += angular_velocity * 0.05)
    {
        pose.pose.position.x = scale_factor * ((R + r) * std::cos(theta) - d * std::cos(((R + r) / r) * theta));
        pose.pose.position.y = scale_factor * ((R + r) * std::sin(theta) - d * std::sin(((R + r) / r) * theta));

        double next_theta = theta + angular_velocity * 0.05;
        double dy = 5 * std::sin(next_theta) * std::cos(next_theta) / (std::pow(std::sin(next_theta), 2) + 1)- pose.pose.position.y;
        double dx = 5 * std::cos(next_theta) / (std::pow(std::sin(next_theta), 2) + 1)- pose.pose.position.x + 1e-5;
        double yaw =  std::atan2(dy, dx);

        tf2::Quaternion quaternion;
        quaternion.setRPY(0, 0, yaw);

        pose.pose.orientation.x = quaternion.x();
        pose.pose.orientation.y = quaternion.y();
        pose.pose.orientation.z = quaternion.z();
        pose.pose.orientation.w = quaternion.w();
        // pose.pose.orientation.w = 1.0; // Assuming orientation is not important for your use case
        path.poses.push_back(pose);
    }

    return path;
}

nav_msgs::msg::Path TrajectoryGenerator::CreateSquarePath()
{
    // std::cout << "Create square path" << std::endl;
    auto path = nav_msgs::msg::Path(); 
    path.header.stamp =  rclcpp::Clock().now();
    path.header.frame_id = "odom";

    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.z = 0.0;

    int period = 1000;
    int l = 10;
    double x = 0.0;
    double y = 0.0;

    for (int t = 0; t < period; t++) {
        if(t <= period * 0.25) {
            x = 0;
            y += l/(period*0.25);
            pose.pose.position.x = x;
            pose.pose.position.y = y;
            SetQuaternion(pose, M_PI_2);
        } else if (t <= period * 0.5) {
            x -= l/(period*0.25);
            pose.pose.position.x = x;
            pose.pose.position.y = y;
            SetQuaternion(pose, M_PI);
        } else if (t <= period * 0.75) {
            y -=  l/(period*0.25);
            pose.pose.position.x = x;
            pose.pose.position.y = y;
            SetQuaternion(pose, -M_PI_2);
        } else if (t <= period) {
            x += l/(period*0.25);
            pose.pose.position.x = x;
            pose.pose.position.y = y;
            SetQuaternion(pose, 0);
        }
        path.poses.push_back(pose);  
    }

    return path;
}

nav_msgs::msg::Path TrajectoryGenerator::CreateInfinitePath()
{
    // std::cout << "Create infinite path" << std::endl;

    auto path = nav_msgs::msg::Path(); 
    path.header.stamp =  rclcpp::Clock().now();
    path.header.frame_id = "odom";

    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.z = 0.0;

    double radius = 5.0; // Radius of the circle
    double frequency = 0.05; // Frequency of publishing points
    double angular_velocity = 2.0 * M_PI * frequency;

    int R = 5;
    int r = 1;
    int d = 3;
    int scale_factor = 1;
    

    for (double theta = 0.0; theta <= 2.0 * M_PI; theta += angular_velocity * 0.05)
    {
        pose.pose.position.x = 10 * std::cos(theta) / (std::pow(std::sin(theta), 2) + 1);
        pose.pose.position.y = 10 * std::sin(theta) * std::cos(theta) / (std::pow(std::sin(theta), 2) + 1);

        double next_theta = theta + angular_velocity * 0.05;
        double dy = 10 * std::cos(next_theta) / (std::pow(std::sin(next_theta), 2) + 1) - pose.pose.position.y;
        double dx = 10 * std::sin(next_theta) * std::cos(next_theta) / (std::pow(std::sin(next_theta), 2) + 1) - pose.pose.position.x + 1e-5;
        double yaw =  std::atan2(dy, dx);

        tf2::Quaternion quaternion;
        quaternion.setRPY(0, 0, yaw);

        pose.pose.orientation.x = quaternion.x();
        pose.pose.orientation.y = quaternion.y();
        pose.pose.orientation.z = quaternion.z();
        pose.pose.orientation.w = quaternion.w();
        // pose.pose.orientation.w = 1.0; // Assuming orientation is not important for your use case
        path.poses.push_back(pose);
    }

    return path;
}

void TrajectoryGenerator::SetQuaternion(geometry_msgs::msg::PoseStamped& pose, const double yaw)
{
    tf2::Quaternion quaternion;
    quaternion.setRPY(0, 0, yaw);

    pose.pose.orientation.x = quaternion.x();
    pose.pose.orientation.y = quaternion.y();
    pose.pose.orientation.z = quaternion.z();
    pose.pose.orientation.w = quaternion.w();
}

}  // namespace waypoint_generator
}  // namespace ros2_controller
