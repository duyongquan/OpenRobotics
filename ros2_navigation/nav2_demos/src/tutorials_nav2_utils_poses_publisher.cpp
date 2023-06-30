#include "nav2_demos/tutorials_nav2_utils_poses_publisher.hpp"

namespace ros2_tutorials
{
namespace nav2
{

PosesPublisher::PosesPublisher(rclcpp::Node* node)
: node_{node}
{
    point_cloud_publisher_ = node->create_publisher<sensor_msgs::msg::PointCloud>("nav2_point_cloud", 10);
    poses_publisher_ = node->create_publisher<geometry_msgs::msg::PoseStamped>("nav2_points", 10);
    path_publisher_ = node->create_publisher<nav_msgs::msg::Path>("nav2_path", 10);
    reference_points_publisher_ = node->create_publisher<geometry_msgs::msg::PoseArray>("reference_points", 10);

    rclcpp::QoS map_qos(10);
    start_sub_ = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "initialpose", map_qos,
            std::bind(&PosesPublisher::HandleStartPointCallback, this, std::placeholders::_1));

    goal_sub_ = node->create_subscription<geometry_msgs::msg::PointStamped>(
        "clicked_point", map_qos,
            std::bind(&PosesPublisher::HandleGoalPointCallback, this, std::placeholders::_1));
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

void PosesPublisher::HandleStartPointCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    if (msg == nullptr) {
        return;
    }

    RCLCPP_INFO(node_->get_logger(), "Receive start initial pose");
    recevie_start_pose_ = true;

    // set pose
    start_.header.frame_id = "map";
    start_.header.stamp = node_->get_clock()->now();
    start_.pose = msg->pose.pose;
}

void PosesPublisher::HandleGoalPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    if (msg == nullptr) {
        return;
    }

    RCLCPP_INFO(node_->get_logger(), "Receive goal target pose");

    // set pose
    goal_.header.frame_id = "map";
    goal_.header.stamp = node_->get_clock()->now();
    goal_.pose.position.x = msg->point.x;
    goal_.pose.position.y = msg->point.y;
    recevie_goal_pose_ = true;
}

bool PosesPublisher::GetStartPoint(geometry_msgs::msg::PoseStamped& start)
{
    if (!recevie_start_pose_) {
        return false;
    }
    start = start_;
    return recevie_start_pose_;
}


bool PosesPublisher::GetGoalPoint(geometry_msgs::msg::PoseStamped& goal)
{
    if (!recevie_goal_pose_) {
        return false;
    }
    goal = goal_;
    return recevie_goal_pose_;
}

}  // namespace nav2
}  // namespace ros2_tutorials
