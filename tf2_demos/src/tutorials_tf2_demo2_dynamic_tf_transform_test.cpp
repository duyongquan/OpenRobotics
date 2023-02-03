
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

namespace ros2_tutorials
{
namespace tf2_demos
{
namespace
{

class Tf2DynamicTransformer : public rclcpp::Node
{
public:
    Tf2DynamicTransformer() : Node("dynamic_tf_transformer")
    {
        // Initialize the transform broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

        // TF2 checker
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
            get_node_base_interface(), 
            get_node_timers_interface());
        tf_buffer_->setCreateTimerInterface(timer_interface);
        tf_buffer_->setUsingDedicatedThread(true);
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        timer_ = create_wall_timer(
            1000ms, std::bind(&Tf2DynamicTransformer::HandleTimerCallback, this));
    }

    ~Tf2DynamicTransformer() {}

private:

    void HandleTimerCallback()
    {
        DoTransform("map", "base_link");
    }

    bool DoTransform(const std::string & parent_link, const std::string & clild_link)
    {
        geometry_msgs::msg::TransformStamped t;

        // Read message content and assign it to
        // corresponding tf variables
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "map";
        t.child_frame_id = "base_link";

        // Turtle only exists in 2D, thus we get x and y translation
        // coordinates from the message and set the z coordinate to 0
        t.transform.translation.x = 1;
        t.transform.translation.y = 1;
        t.transform.translation.z = 0.0;

        // For the same reason, turtle can only rotate around one axis
        // and this why we set rotation in x and y to 0 and obtain
        // rotation in z axis from the message
        tf2::Quaternion q;
        q.setRPY(0, 0, 1.0);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        // Send the transformation
        tf_broadcaster_->sendTransform(t);
        return true;
    }

    rclcpp::TimerBase::SharedPtr timer_ {nullptr};
    // tf
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_{nullptr};
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_{nullptr};
};
}

}
}

/**
 * Usage:
 * 
 *  ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 base_link map
 * 
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ros2_tutorials::tf2_demos::Tf2DynamicTransformer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
