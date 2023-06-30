// Copyright (c) 2011, Willow Garage, Inc.
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Willow Garage, Inc. nor the names of its
//       contributors may be used to endorse or promote products derived from
//       this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <memory>
#include <string>
#include <vector>
#include <deque>

#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "interactive_markers/interactive_marker_server.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/interactive_marker.hpp"
#include "visualization_msgs/msg/interactive_marker_control.hpp"
#include "visualization_msgs/msg/interactive_marker_feedback.hpp"
#include "visualization_msgs/msg/marker.hpp"

using namespace std::chrono_literals;

namespace ros2_tutorials
{
namespace rviz
{

class EightPathGenerator : public rclcpp::Node
{
public:
    explicit EightPathGenerator();
    ~EightPathGenerator() = default;

private:
    void HandleTimerCallback();

    void HandlePathTypeCallback(const std_msgs::msg::Int32::SharedPtr msg);

    nav_msgs::msg::Path CreateLissajousPath();

    nav_msgs::msg::Path CreateCirclePath();

    nav_msgs::msg::Path CreateRectanglePath();

    void PublishTargatPose(const geometry_msgs::msg::PoseStamped& pose);

    // tf
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // timer
    rclcpp::TimerBase::SharedPtr timer_ {nullptr};

    // publisher
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pos_pub_ {nullptr};
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_ {nullptr};
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_{nullptr};

    // subscription
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr cmd_path_type_sub_{nullptr};

    bool load_waypoint_finished_{false};
    std::deque<geometry_msgs::msg::PoseStamped> poses_;

    int path_type_ = 1;
};  // class EightPathGenerator


EightPathGenerator::EightPathGenerator() : Node("eight_path_node")
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
    pos_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        "target_point", rclcpp::SystemDefaultsQoS());

    path_publisher_ = create_publisher<nav_msgs::msg::Path>(
        "global_path", rclcpp::SystemDefaultsQoS());

    marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "pose", 1);

    // subscription
    cmd_path_type_sub_ = this->create_subscription<std_msgs::msg::Int32>(
        "path_type", 10, std::bind(&EightPathGenerator::HandlePathTypeCallback, this, std::placeholders::_1));

    // timer
    timer_ = create_wall_timer(
        1000ms, std::bind(&EightPathGenerator::HandleTimerCallback, this));
}

void EightPathGenerator::HandlePathTypeCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
    if (msg == nullptr) {
        return;
    }

    poses_.clear();
    load_waypoint_finished_ = false;

    if (msg->data == 1) {
        path_type_ = 1;
    } else if (msg->data == 2) {
        path_type_ = 2;
    } else if (msg->data == 3) {
        path_type_ = 3;
    }
}

void EightPathGenerator::HandleTimerCallback()
{
    if (!load_waypoint_finished_) {
        auto path = nav_msgs::msg::Path();
        if (path_type_ == 1) {
            // Lissajous path
            path = CreateLissajousPath();
        } else if (path_type_ == 2) {
            // Circle path
            path = CreateCirclePath();
        } else if (path_type_ == 3) {
            // Rectangle path
            path = CreateRectanglePath();
        }
    
        RCLCPP_INFO(this->get_logger(), "path size: %d", path.poses.size());
        path_publisher_->publish(path);
    }

    if (poses_.empty()) {
        load_waypoint_finished_ = false;
        return;
    }

    auto pose = poses_.front();
    poses_.pop_front();
    PublishTargatPose(pose);
}

nav_msgs::msg::Path EightPathGenerator::CreateLissajousPath()
{
    const double pi_2 = 6.28;
    auto path = nav_msgs::msg::Path();
    path.header.stamp =  this->get_clock()->now();
    path.header.frame_id = "odom";

    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.z = 0.0;

    double radius = 10.0f;

    path.poses.clear();
    for (std::size_t i = 0; i < 2000; i++) {
        double theta_x = (i * pi_2);
        double theta_y = (2 * i * pi_2);
        double x = radius * std::sin(theta_x);
        double y = radius * std::sin(theta_y);

        if (i % 10 == 0) {
            pose.pose.position.x = x;
            pose.pose.position.y = y;
            path.poses.push_back(pose);
        }
    }

    // load waypoints
    for (auto pose : path.poses) {
        poses_.push_back(pose);
    }

    load_waypoint_finished_ = true;
    return path;
}

nav_msgs::msg::Path EightPathGenerator::CreateCirclePath()
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

    // load waypoints
    for (auto pose : path.poses) {
        poses_.push_back(pose);
    }

    load_waypoint_finished_ = true;
    return path;
}

nav_msgs::msg::Path EightPathGenerator::CreateRectanglePath()
{
    auto path = nav_msgs::msg::Path();
    path.header.stamp =  this->get_clock()->now();
    path.header.frame_id = "odom";

    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.z = 0.0;

    double width = 5.0;
    double height = 8.0;
    double step = 0.1;
    int W = static_cast<int>(width / step);
    int H = static_cast<int>(height / step);

    for (int i = 0; i < W; i++) {
        double x =  i * width / W;
        double y = 0.0;

        pose.pose.position.x = x;
        pose.pose.position.y = y;
        path.poses.push_back(pose);
    }

    for (int i = 0; i < H; i++) {
        double x =  width;
        double y = i * width / H;

        pose.pose.position.x = x;
        pose.pose.position.y = y;
        path.poses.push_back(pose);
    }

    for (int i = W; i > 0; i--) {
        double x = i * width / W;
        double y = height;

        pose.pose.position.x = x;
        pose.pose.position.y = y;
        path.poses.push_back(pose);
    }

    for (int i = H; i > 0; i--) {
        double x = 0;
        double y = i * height / H;

        pose.pose.position.x = x;
        pose.pose.position.y = y;
        path.poses.push_back(pose);
    }

    // load waypoints
    for (auto pose : path.poses) {
        poses_.push_back(pose);
    }

    load_waypoint_finished_ = true;
    return path;

    return path;
}

void EightPathGenerator::PublishTargatPose(const geometry_msgs::msg::PoseStamped& pose)
{
    // Set our initial shape type to be a cube
    uint32_t shape = visualization_msgs::msg::Marker::CUBE;

    visualization_msgs::msg::Marker marker;
    // Set the frame ID and timestamp. See the TF tutorials for information on these.
    marker.header.frame_id = "odom";
    marker.header.stamp = rclcpp::Clock().now();

    // Set the namespace and id for this marker. This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type
    // Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::msg::Marker::SPHERE;

    // Set the marker action
    // Options are ADD, DELETE, and DELETEALL
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Set the pose of the marker
    // This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose = pose.pose;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.2f;
    marker.color.g = 0.1f;
    marker.color.b = 0.7f;
    marker.color.a = 1.0;

    // Set the lifetime of the marker -- 0 indicates forever
    marker.lifetime = rclcpp::Duration::from_nanoseconds(0);

    // Publish the marker
    marker_publisher_->publish(marker);
}

}  // namespace rviz
}  // namespace ros2_tutorials




int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ros2_tutorials::rviz::EightPathGenerator>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
