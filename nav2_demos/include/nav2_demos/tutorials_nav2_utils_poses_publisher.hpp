#ifndef ROS2_TUTORIALS__TUTORIALS_NAV2_UTILS_POSES_PUBLISHER_HPP_
#define ROS2_TUTORIALS__TUTORIALS_NAV2_UTILS_POSES_PUBLISHER_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using namespace std::chrono_literals;

namespace ros2_tutorials
{
namespace nav2
{

class PosesPublisher
{
public:
    PosesPublisher(rclcpp::Node* node);
    ~PosesPublisher();

    void PublishPoses(const std::vector<geometry_msgs::msg::PoseStamped>& points);

    void PublishPoses(const sensor_msgs::msg::PointCloud2& points);

    void PublishPoses(const sensor_msgs::msg::PointCloud& points);

    void PublishPath(const nav_msgs::msg::Path& path);

    void PublishReferencePoses(const geometry_msgs::msg::PoseArray& reference_points);

    bool GetStartPoint(geometry_msgs::msg::PoseStamped& start);

    bool GetGoalPoint(geometry_msgs::msg::PoseStamped& goal);

private:
    void HandleStartPointCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void HandleGoalPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);

    rclcpp::Node* node_{nullptr};
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr poses_publisher_ {nullptr};
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr point_cloud_publisher_ {nullptr};
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud2_publisher_ {nullptr};
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_ {nullptr};
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr reference_points_publisher_ {nullptr};

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr start_sub_{nullptr};
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_{nullptr};
    geometry_msgs::msg::PoseStamped start_;
    geometry_msgs::msg::PoseStamped goal_;
    bool recevie_start_pose_{false};
    bool recevie_goal_pose_{false};
};

}  // namespace nav2
}  // namespace ros2_tutorials

#endif  // ROS2_TUTORIALS__TUTORIALS_NAV2_UTILS_POSES_PUBLISHER_HPP_