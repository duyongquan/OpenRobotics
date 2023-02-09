#include <chrono>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/path.hpp"

#include "nav2_demos/tutorials_nav2_utils_poses_publisher.hpp"

using namespace std::chrono_literals;

namespace ros2_tutorials
{
namespace nav2
{
namespace
{

class BezierPathNode : public rclcpp::Node
{
public:
    BezierPathNode() : Node("bezier_path")
    {
        pose_publisher_ = std::make_shared<PosesPublisher>(this);
        timer_ = create_wall_timer(
            1000ms, std::bind(&BezierPathNode::HandleTimerCallback, this));
    }

    ~BezierPathNode() {}

private:

    void HandleTimerCallback()
    {
        auto points = CreatePoints();
        pose_publisher_->PublishPoses(points);
    }

    sensor_msgs::msg::PointCloud CreatePoints()
    {
        sensor_msgs::msg::PointCloud points;
        points.header.frame_id = "map";
        points.header.stamp = this->get_clock()->now();

        for (int i = 0; i < 10; i++) {
            geometry_msgs::msg::Point32 point;
            point.x = i;
            point.y = pow(i, 2);
            points.points.push_back(point);
        }
        return points;
    }

    rclcpp::TimerBase::SharedPtr timer_ {nullptr};
    std::shared_ptr<PosesPublisher> pose_publisher_ {nullptr};
};  
}


}  // namespace nav2
}  // namespace ros2_tutorials



int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ros2_tutorials::nav2::BezierPathNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
