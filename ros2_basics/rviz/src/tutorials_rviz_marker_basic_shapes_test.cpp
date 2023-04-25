#include <memory>

#include "rclcpp/logging.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

using namespace std::chrono_literals;

namespace ros2_tutorials
{
namespace rviz
{

class BasicShapes : public rclcpp::Node
{
public:
    BasicShapes() : Node("basic_shapes")
    {
        marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "visualization_marker", 1);

        timer_ = this->create_wall_timer(1s, 
            std::bind(&BasicShapes::HandleTimerCallback, this));
    }

    ~BasicShapes() {}

private:
    void HandleTimerCallback()
    {
        // Set our initial shape type to be a cube
        uint32_t shape = visualization_msgs::msg::Marker::CUBE;

        visualization_msgs::msg::Marker marker;
        // Set the frame ID and timestamp. See the TF tutorials for information on these.
        marker.header.frame_id = "/my_frame";
        marker.header.stamp = rclcpp::Clock().now();

        // Set the namespace and id for this marker. This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        marker.ns = "basic_shapes";
        marker.id = 0;

        // Set the marker type
        // Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
        marker.type = shape;

        // Set the marker action
        // Options are ADD, DELETE, and DELETEALL
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Set the pose of the marker
        // This is a full 6DOF pose relative to the frame/time specified in the header
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;

        // Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        // Set the lifetime of the marker -- 0 indicates forever
        marker.lifetime = rclcpp::Duration::from_nanoseconds(0);

        // Publish the marker
        marker_publisher_->publish(marker);

        // Cycle between different shapes
        switch (shape)
        {
            case visualization_msgs::msg::Marker::CUBE:
                shape = visualization_msgs::msg::Marker::SPHERE;
                break;
            case visualization_msgs::msg::Marker::SPHERE:
                shape = visualization_msgs::msg::Marker::ARROW;
                break;
            case visualization_msgs::msg::Marker::ARROW:
                shape = visualization_msgs::msg::Marker::CYLINDER;
                break;
            case visualization_msgs::msg::Marker::CYLINDER:
                shape = visualization_msgs::msg::Marker::CUBE;
                break;
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
};

}  // namespace rviz
}  // namespace ros2_tutorials

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ros2_tutorials::rviz::BasicShapes>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
