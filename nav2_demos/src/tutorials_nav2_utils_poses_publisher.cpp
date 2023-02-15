#include "nav2_demos/tutorials_nav2_utils_poses_publisher.hpp"

namespace ros2_tutorials
{
namespace nav2
{

PosesPublisher::PosesPublisher(rclcpp::Node* node)
{
    point_cloud_publisher_ = node->create_publisher<sensor_msgs::msg::PointCloud>("nav2_point_cloud", 10);
    poses_publisher_ = node->create_publisher<geometry_msgs::msg::PoseStamped>("nav2_points", 10);
    path_publisher_ = node->create_publisher<nav_msgs::msg::Path>("nav2_path", 10);
    reference_points_publisher_ = node->create_publisher<geometry_msgs::msg::PoseArray>("reference_points", 10);
}

PosesPublisher::~PosesPublisher()
{

}


void PosesPublisher::PublishPoses(const std::vector<geometry_msgs::msg::PoseStamped>& poinits)
{
    
}

void PosesPublisher::PublishPoses(const sensor_msgs::msg::PointCloud2& points)
{

}

void PosesPublisher::PublishPoses(const sensor_msgs::msg::PointCloud& points)
{
    point_cloud_publisher_->publish(points);
}

void PosesPublisher::PublishPath(const nav_msgs::msg::Path& path)
{
    path_publisher_->publish(path);
}

void PosesPublisher::PublishReferencePoses(const geometry_msgs::msg::PoseArray& reference_points)
{
    reference_points_publisher_->publish(reference_points);
}

}  // namespace nav2
}  // namespace ros2_tutorials
