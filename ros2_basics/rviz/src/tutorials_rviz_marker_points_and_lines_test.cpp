#include <memory>

#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

using namespace std::chrono_literals;

namespace ros2_tutorials
{
namespace rviz
{

class PointsAndLines : public rclcpp::Node
{
public:
    PointsAndLines() : Node("points_and_lines")
    {
        marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "visualization_marker", 1);

        timer_ = this->create_wall_timer(1s, 
            std::bind(&PointsAndLines::HandleTimerCallback, this));
    }

    ~PointsAndLines() {}

private:
    void HandleTimerCallback()
    {
        visualization_msgs::msg::Marker points, line_strip, line_list;
        points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "/map";
        points.header.stamp = line_strip.header.stamp = line_list.header.stamp = rclcpp::Clock().now();
        points.ns = line_strip.ns = line_list.ns = "points_and_lines";
        points.action = line_strip.action = line_list.action = visualization_msgs::msg::Marker::ADD;

        points.id = 0;
        line_strip.id = 1;
        line_list.id = 2;

        points.type = visualization_msgs::msg::Marker::POINTS;
        line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line_list.type = visualization_msgs::msg::Marker::LINE_LIST;

        // POINTS markers use x and y scale for width/height respectively
        points.scale.x = 0.2;
        points.scale.y = 0.2;

        // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
        line_strip.scale.x = 0.1;
        line_list.scale.x = 0.1;

        // Points are green
        points.color.g = 1.0f;
        points.color.a = 1.0;

        // Line strip is blue
        line_strip.color.b = 1.0;
        line_strip.color.a = 1.0;

        // Line list is red
        line_list.color.r = 1.0;
        line_list.color.a = 1.0;

        // Create the vertices for the points and lines
        for (uint32_t i = 0; i < 100; ++i) {
            float y = 5 * sin(f + i / 100.0f * 2 * M_PI);
            float z = 5 * cos(f + i / 100.0f * 2 * M_PI);

            geometry_msgs::msg::Point p;
            p.x = (int32_t)i - 50;
            p.y = y;
            p.z = z;

            points.points.push_back(p);
            line_strip.points.push_back(p);

            // The line list needs two points for each line
            line_list.points.push_back(p);
            p.z += 1.0;
            line_list.points.push_back(p);
        }

        marker_publisher_->publish(points);
        marker_publisher_->publish(line_strip);
        marker_publisher_->publish(line_list);

        f += 0.04f;
    }

    float f = {0.0f};
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
};

}  // namespace rviz
}  // namespace ros2_tutorials

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ros2_tutorials::rviz::PointsAndLines>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
