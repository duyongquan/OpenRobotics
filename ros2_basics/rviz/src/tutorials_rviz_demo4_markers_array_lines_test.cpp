#include <memory>

#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;

namespace ros2_tutorials
{
namespace rviz
{

class MarkerArrayLines : public rclcpp::Node
{
public:
    MarkerArrayLines() : Node("point_lines")
    {
        marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "visualization_marker", 1);

        timer_ = this->create_wall_timer(1s, 
            std::bind(&MarkerArrayLines::HandleTimerCallback, this));
    }

    ~MarkerArrayLines() {}

private:
    void HandleTimerCallback()
    {
        std::vector<geometry_msgs::msg::Point> points;
        geometry_msgs::msg::Point start, end;
        start.x = 0;
        start.y = 0;
        end.x = 2.5;
        end.y = 2.5;
        points.push_back(start);
        points.push_back(end);

        // point2
        end.x = 2.5;
        end.y = -2.5;
        points.push_back(start);
        points.push_back(end);

        // point3
        end.x = -2.5;
        end.y = -2.5;
        points.push_back(start);
        points.push_back(end);

        // point4
        end.x = -2.5;
        end.y = 2.5;
        points.push_back(start);
        points.push_back(end);

        PubilishRaycastLines(points);
    }

    void PubilishRaycastLines(const std::vector<geometry_msgs::msg::Point> & points)
    {
        if (points.empty()) {
            RCLCPP_ERROR(this->get_logger(), "The points is empty");
        }

        visualization_msgs::msg::Marker point_lines;
        point_lines.ns = "line_mark";   
        point_lines.type = visualization_msgs::msg::Marker::LINE_LIST;     
        point_lines.pose.orientation.x=0.0;
        point_lines.pose.orientation.y=0.0;
        point_lines.pose.orientation.z=0.0;
        point_lines.pose.orientation.w=1.0;
        point_lines.scale.x = 0.1;
        point_lines.color.r = 1.0;
        point_lines.color.g = 0.0;
        point_lines.color.b = 0.0;
        point_lines.color.a = 1.0;

        int count = 0;
        visualization_msgs::msg::MarkerArray lines_marker;

        for(long unsigned int i = 0; i < points.size(); i++) {                                                     
            point_lines.id = count;
            geometry_msgs::msg::Point p_start;
            p_start.x = points[2*i].x;
            p_start.y = points[2*i].y;
            p_start.z = 0.0;	
            point_lines.points.push_back(p_start);
            geometry_msgs::msg::Point p_end;
            p_end.x = points[2*i + 1].x;
            p_end.y = points[2*i + 1].y;
            p_end.z = 0.0;
            point_lines.points.push_back(p_end);
                
            point_lines.header.frame_id = "map";
            point_lines.header.stamp = rclcpp::Clock().now();
            lines_marker.markers.push_back(point_lines);
            point_lines.points.clear();
            count++;
        }
        marker_publisher_->publish(lines_marker);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
};

}  // namespace rviz
}  // namespace ros2_tutorials

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ros2_tutorials::rviz::MarkerArrayLines>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
