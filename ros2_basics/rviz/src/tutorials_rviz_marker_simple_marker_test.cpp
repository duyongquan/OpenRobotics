#include <memory>
#include <sstream>

#include "interactive_markers/interactive_marker_server.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/interactive_marker.hpp"
#include "visualization_msgs/msg/interactive_marker_control.hpp"
#include "visualization_msgs/msg/marker.hpp"

using namespace std::chrono_literals;

namespace ros2_tutorials
{
namespace rviz
{

class SimpleMarker : public rclcpp::Node
{
public:
    SimpleMarker() : Node("simple_marker")
    {
        server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
            "simple_marker", this);
        
        // create an interactive marker for our server
        visualization_msgs::msg::InteractiveMarker interactive_marker;
        interactive_marker.header.frame_id = "map";
        interactive_marker.header.stamp = this->get_clock()->now();
        interactive_marker.name = "my_marker";
        interactive_marker.description = "Simple 1-DOF Control";

        // create a grey box marker
        visualization_msgs::msg::Marker box_marker;
        box_marker.type = visualization_msgs::msg::Marker::CUBE;
        box_marker.scale.x = 0.45;
        box_marker.scale.y = 0.45;
        box_marker.scale.z = 0.45;
        box_marker.color.r = 0.5;
        box_marker.color.g = 0.5;
        box_marker.color.b = 0.5;
        box_marker.color.a = 1.0;

        // create a non-interactive control which contains the box
        visualization_msgs::msg::InteractiveMarkerControl box_control;
        box_control.always_visible = true;
        box_control.markers.push_back(box_marker);

        // add the control to the interactive marker
        interactive_marker.controls.push_back(box_control);

        // create a control which will move the box
        // this control does not contain any markers,
        // which will cause RViz to insert two arrows
        visualization_msgs::msg::InteractiveMarkerControl rotate_control;
        rotate_control.name = "move_x";
        rotate_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;

        // add the control to the interactive marker
        interactive_marker.controls.push_back(rotate_control);

        // add the interactive marker to our collection &
        // tell the server to call HandleProcessCallback() when feedback arrives for it
        server_->insert(interactive_marker,
            std::bind(&SimpleMarker::HandleProcessCallback, this, std::placeholders::_1));

        // 'commit' changes and send to all clients
        server_->applyChanges();

        RCLCPP_INFO(this->get_logger(), "Ready");
    }

    ~SimpleMarker() {}

private:
    void HandleProcessCallback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback)
    {
        std::ostringstream oss;
        oss << feedback->marker_name << " is now at " 
            << feedback->pose.position.x << ", " 
            << feedback->pose.position.y << ", " 
            << feedback->pose.position.z;

        RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
    }

    // create an interactive marker server on the namespace "simple_marker"
    std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_{nullptr};
};

}  // namespace rviz
}  // namespace ros2_tutorials

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ros2_tutorials::rviz::SimpleMarker>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
