#include "waypoint_generator/bernoulli_curve.hpp"

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include <cmath>

namespace ros2_controller {
namespace waypoint_generator {
namespace {

class PathVisualizerNode : public rclcpp::Node
{
public:
    PathVisualizerNode()
        : Node("path_visualizer_node")
    {
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("path_marker", 10);
        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("plan", rclcpp::SystemDefaultsQoS());
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&PathVisualizerNode::timer_callback, this));
    }

    void timer_callback()
    {
        auto plan = lemniscate_bernoulli::CreatePath(2);
        RCLCPP_INFO(this->get_logger(), "path size: %d", plan.poses.size());
        // path_publisher_->publish(plan);

        auto marker_msg = ToMarker(plan);
        marker_pub_->publish(marker_msg);
    }

    visualization_msgs::msg::Marker ToMarker(const nav_msgs::msg::Path& path)
    {
        auto marker_msg = visualization_msgs::msg::Marker();
        marker_msg.header.frame_id = "odom";
        marker_msg.header.stamp = rclcpp::Clock().now();
        marker_msg.ns = "path";
        marker_msg.id = 0;
        marker_msg.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker_msg.action = visualization_msgs::msg::Marker::ADD;

        marker_msg.scale.x = 0.02;  // Line width

        marker_msg.color.r = 1.0;
        marker_msg.color.g = 0.0;
        marker_msg.color.b = 0.0;
        marker_msg.color.a = 1.0;

        marker_msg.points.clear();

        for (auto pose : path.poses) {
            geometry_msgs::msg::Point point;
            point.x =  pose.pose.position.x;
            point.y = pose.pose.position.y;
            point.z = 0.0;
            marker_msg.points.push_back(point);
        }

        return marker_msg;
    }

    nav_msgs::msg::Path CreateCirclePath()
    {
        const double pi_2 = 6.28;
        auto path = nav_msgs::msg::Path();

        path.header.stamp =  this->get_clock()->now();
        path.header.frame_id = "odom";

        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position.z = 0.0;

        // circle
        for (std::size_t i = 0; i < 1000 ; i++) {

            double theta =  (i * pi_2 )/ 1000;
            double radius = 10.0f;

            double x = radius * std::cos(theta);
            double y = radius * std::sin(theta);
            pose.pose.position.x = x;
            pose.pose.position.y = y;

            if (i % 10 == 0) {
                pose.pose.position.x = x;
                pose.pose.position.y = y;
                path.poses.push_back(pose);
            }
        }

        return path;
    }


private:
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_{nullptr};
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_{nullptr};
    rclcpp::TimerBase::SharedPtr timer_{nullptr};
};


}  // namespace 
}  // namespace waypoint_generator
}  // namespace ros2_controller


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto waypoint_generator = std::make_shared<ros2_controller::waypoint_generator::PathVisualizerNode>();
  rclcpp::spin(waypoint_generator);
  rclcpp::shutdown();
  return 0;
}
