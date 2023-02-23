
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/robot_utils.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;

namespace ros2_tutorials
{
namespace timer
{

class PosePublisher : public rclcpp::Node
{
public:
  PosePublisher() : Node("pose_report")
  {
    // tf
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        get_node_base_interface(),
        get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_buffer_->setUsingDedicatedThread(true);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // pose publisher
    global_pose_topic_ = "dog_pose";
    pos_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        global_pose_topic_, rclcpp::SystemDefaultsQoS());

    // command control
    subscription_  = this->create_subscription<std_msgs::msg::Int32>(
        "pose_cmd", 10, std::bind(&PosePublisher::HandleCommandCallback, this, std::placeholders::_1));

    // timer
    timer_ = create_wall_timer(
        1000ms, std::bind(&PosePublisher::HandleGloablPoseCallback, this));
  }

  ~PosePublisher() {}

  void Start()
  {
    start_ = true;
    stop_ = false;
    timer_->execute_callback();
  }

  void Close()
  {
    start_ = false;
    stop_ = true;
    timer_->cancel();
  }

  bool IsStart()
  {
    return start_;
  }

  bool IsStop()
  {
    return start_;
  }

private:
  void HandleGloablPoseCallback()
  {
    if (start_) {
        geometry_msgs::msg::PoseStamped gloabl_pose;
        if (!nav2_util::getCurrentPose(gloabl_pose, *tf_buffer_, "map", "base_link", 2.0)) {
            RCLCPP_WARN(this->get_logger(), "Failed to obtain current pose based on map coordinate system.");
        } else {
            pos_pub_->publish(gloabl_pose);
        }
    }
  }

  void HandleCommandCallback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "command : %d", msg->data);
    if (msg->data == 0) {
        RCLCPP_INFO(this->get_logger(), "start ...  ");
        // this->Start();
        start_ = true;
        stop_ = false;
    }

    if (msg->data == 1) {
        RCLCPP_ERROR(this->get_logger(), "stop...  ");
        // this->Close();
        start_ = false;
        stop_ = true;
    }
  }

  std::string global_pose_topic_;
  rclcpp::TimerBase::SharedPtr timer_ {nullptr};

  // publisher
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pos_pub_ {nullptr};
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_ {nullptr};

  // tf
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // flags
  bool start_ {false};
  bool stop_ {false};
};

}  // namespace timer
}  // namespace ros2_tutorials

/**
 * Usage:
 * 
 *  ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 base_link map
 * 
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ros2_tutorials::timer::PosePublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
