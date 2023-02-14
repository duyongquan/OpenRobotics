#include <chrono>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/path.hpp"

#include "spline.h"
#include "nav2_demos/tutorials_nav2_utils_poses_publisher.hpp"

using namespace std::chrono_literals;

namespace ros2_tutorials
{
namespace nav2
{
namespace
{

class BSplinePathNode : public rclcpp::Node
{
public:
    BSplinePathNode() : Node("b_spline_path")
    {
        pose_publisher_ = std::make_shared<PosesPublisher>(this);
        timer_ = create_wall_timer(
            1000ms, std::bind(&BSplinePathNode::HandleTimerCallback, this));
    }

    ~BSplinePathNode() {}

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

        std::vector<double> way_point_x = {-0.1, 0.4, 1.2, 1.8, 2.0};;
        std::vector<double> way_point_y = {0.1, 0.7, 0.6, 1.1, 0.9};

        for (int i = 0; i < way_point_x.size(); i++) {
            geometry_msgs::msg::Point32 point;
            point.x = way_point_x[i];
            point.y = way_point_y[i];
            points.points.push_back(point);
        }
        return points;
    }

    void ApproximateBSplinePath(const sensor_msgs::msg::PointCloud& points, 
        int n_course_point, int smooth_weight)
    {
    }

    rclcpp::TimerBase::SharedPtr timer_ {nullptr};
    std::shared_ptr<PosesPublisher> pose_publisher_ {nullptr};
    // boost::math::interpolators::cardinal_quadratic_b_spline<double> spline_;
};  
}

}  // namespace nav2WW
}  // namespace ros2_tutorials


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ros2_tutorials::nav2::BSplinePathNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
