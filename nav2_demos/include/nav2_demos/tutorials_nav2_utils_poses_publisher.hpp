#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
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

private:
    rclcpp::Node::SharedPtr node_{nullptr};
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr poses_publisher_ {nullptr};
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr point_cloud_publisher_ {nullptr};
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud2_publisher_ {nullptr};
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_ {nullptr};
};

}  // namespace nav2
}  // namespace ros2_tutorials

