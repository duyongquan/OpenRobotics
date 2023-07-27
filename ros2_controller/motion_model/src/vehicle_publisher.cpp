#include "motion_model/vehicle_publisher.hpp"

namespace ros2_controller
{
namespace motion_model
{

VehiclePositionPublisher::VehiclePositionPublisher()
    : rclcpp::Node("vehiclposition_publisher")
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

    vel_publisher_ =  this->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/smart/velocity", 1);

    rear_pose_publisher_ =  this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/smart/rear_pose", 1);

    center_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/smart/center_pose", 1);
 
    pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/smart/center_pose", 10, 
        std::bind(&VehiclePositionPublisher::HandleGazeboPoseCallback, this, std::placeholders::_1));

    model_states_subscription_ = this->create_subscription<gazebo_msgs::msg::ModelStates>(
        "/gazebo/model_states", 10, 
        std::bind(&VehiclePositionPublisher::HandleGazeboModelStatesPoseCallback, this, std::placeholders::_1));

}

void VehiclePositionPublisher::HandleGazeboPoseCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) const
{
    geometry_msgs::msg::TransformStamped pose;

    // Read message content and assign it to
    // corresponding tf variables
    // pose.header.stamp = this->get_clock()->now();
    pose.header.frame_id = "world";
    pose.child_frame_id = "base_link";

    // Turtle only exists in 2D, thus we get x and y translation
    // coordinates from the message and set the z coordinate to 0
    pose.transform.translation.x = msg->pose.position.x;
    pose.transform.translation.y = msg->pose.position.y;
    pose.transform.translation.z = msg->pose.position.z;

    // For the same reason, turtle can only rotate around one axis
    // and this why we set rotation in x and y to 0 and obtain
    // rotation in z axis from the message
    pose.transform.rotation = msg->pose.orientation;

    // Send the transformation
    tf_broadcaster_->sendTransform(pose);
}

void VehiclePositionPublisher::HandleGazeboModelStatesPoseCallback(
    const gazebo_msgs::msg::ModelStates::SharedPtr msg) const
{
    // auto vehicle_name = msg->name;
    // msg->name;

    int index = 0;
    bool find = false;
    for (; index < msg->name.size(); index++) {
        if (msg->name[index] == "smart") {
            find = true;
            break;
        }
    }
    
    if (!find)
        return;
    geometry_msgs::msg::Pose vehicle_position = msg->pose[index];
    geometry_msgs::msg::Twist vehicle_velocity = msg->twist[index];
    geometry_msgs::msg::Quaternion orientation = vehicle_position.orientation;

    tf2::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // vehicle center position
    geometry_msgs::msg::PoseStamped center_pose;
    center_pose.header.frame_id = '/world';
    center_pose.header.stamp = rclcpp::Clock().now();
    center_pose.pose.position = vehicle_position.position;
    center_pose.pose.orientation = vehicle_position.orientation;
    center_pose_publisher_->publish(center_pose);

    // vehicle rear axle position
    geometry_msgs::msg::PoseStamped rear_pose;
    rear_pose.header.frame_id = '/world';
    rear_pose.header.stamp = rclcpp::Clock().now();
    rear_pose.pose.position.x = vehicle_position.position.x - cos(yaw) * 0.945;
    rear_pose.pose.position.y = vehicle_position.position.y - sin(yaw) * 0.945;
    rear_pose.pose.orientation = vehicle_position.orientation;
    rear_pose_publisher_->publish(rear_pose);

    // vehicle velocity
    geometry_msgs::msg::TwistStamped velocity;
    velocity.header.frame_id = "base_link";
    velocity.header.stamp = rclcpp::Clock().now();
    velocity.twist.linear = vehicle_velocity.linear;
    velocity.twist.angular = vehicle_velocity.angular;
    vel_publisher_->publish(velocity);

}

}  // namespace motion_model
}  // namespace ros2_controller