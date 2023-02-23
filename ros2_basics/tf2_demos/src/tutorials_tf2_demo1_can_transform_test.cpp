
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

namespace ros2_tutorials
{
namespace tf2_demos
{

class Tf2Transformer : public rclcpp::Node
{
public:
    Tf2Transformer() : Node("tf_transformer")
    {
        // TF2 checker
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
            get_node_base_interface(), 
            get_node_timers_interface());
        tf_buffer_->setCreateTimerInterface(timer_interface);
        tf_buffer_->setUsingDedicatedThread(true);
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        timer_ = create_wall_timer(
            1000ms, std::bind(&Tf2Transformer::HandleTimerCallback, this));
    }

    ~Tf2Transformer() {}

private:

    void HandleTimerCallback()
    {
        bool result = CanTransform("map", "base_link");
        if (result) {
            RCLCPP_INFO(this->get_logger(), "can transform from map to base_link");
        } else {
            RCLCPP_ERROR(this->get_logger(), "can't transform from map to base_link");
        }
    }

    bool CanTransform(const std::string & parent_link, const std::string & clild_link)
    {
        // Look up for the transformation between parent_link and clild_link frames
        return tf_buffer_->canTransform(parent_link, clild_link, rclcpp::Time());

        // bool result = true;
        // try {
        //     tf_buffer_->lookupTransform(clild_link, parent_link, rclcpp::Time());

        //      std::this_thread::sleep_for(std::chrono::milliseconds(100));
        // } catch (tf2::TransformException & ex) {
        //     result = false;
        // }

        // return result;
    }

    rclcpp::TimerBase::SharedPtr timer_ {nullptr};
    // tf
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_{nullptr};
};

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
  auto node = std::make_shared<ros2_tutorials::tf2_demos::Tf2Transformer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
