#include "quad_teleop/teleop_twist_joy.hpp"

#include <map>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/u_int8.hpp"

using namespace std::chrono_literals;

namespace ros2_quadruped {
namespace quad_teleop {

struct TeleopTwistJoy::Impl 
{
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy);
    void sendCmdVelMsg(const sensor_msgs::msg::Joy::SharedPtr joy_msg, const std::string& which_map);

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub{nullptr};
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub{nullptr};
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr control_mode_pub{nullptr};

    int enable_button;
    int enable_turbo_button;
    int stand_button;
    int sit_button;
    int safety_button;

    std::map<std::string, int> axis_linear_map;
    std::map<std::string, std::map<std::string, double> > scale_linear_map;

    std::map<std::string, int> axis_angular_map;
    std::map<std::string, std::map<std::string, double> > scale_angular_map;

    bool sent_disable_msg;
};

TeleopTwistJoy::TeleopTwistJoy() 
    : Node("quad_teleop_node")
{
    pimpl_ = new Impl;

    // std::string control_mode_topic;
    // nh_param->param<std::string>("control/mode", control_mode_topic, "/control/mode");
    // pimpl_->control_mode_pub = nh->advertise<std_msgs::UInt8>(control_mode_topic, 1, false);
    // pimpl_->cmd_vel_pub = nh->advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
    // pimpl_->joy_sub = nh->subscribe<sensor_msgs::Joy>("joy", 1, &TeleopTwistJoy::Impl::joyCallback, pimpl_);

    // nh_param->param<int>("enable_button", pimpl_->enable_button, 0);
    // nh_param->param<int>("enable_turbo_button", pimpl_->enable_turbo_button, -1);

    // nh_param->param<int>("stand_button", pimpl_->stand_button, -1);
    // nh_param->param<int>("sit_button", pimpl_->sit_button, -1);
    // nh_param->param<int>("safety_button", pimpl_->safety_button, -1);

    // if (nh_param->getParam("axis_linear", pimpl_->axis_linear_map)) {
    //     nh_param->getParam("scale_linear", pimpl_->scale_linear_map["normal"]);
    //     nh_param->getParam("scale_linear_turbo", pimpl_->scale_linear_map["turbo"]);
    // } else {
    //     nh_param->param<int>("axis_linear", pimpl_->axis_linear_map["x"], 1);
    //     nh_param->param<double>("scale_linear", pimpl_->scale_linear_map["normal"]["x"], 0.5);
    //     nh_param->param<double>("scale_linear_turbo", pimpl_->scale_linear_map["turbo"]["x"], 1.0);
    // }

    // if (nh_param->getParam("axis_angular", pimpl_->axis_angular_map)) {
    //     nh_param->getParam("scale_angular", pimpl_->scale_angular_map["normal"]);
    //     nh_param->getParam("scale_angular_turbo", pimpl_->scale_angular_map["turbo"]);
    // } else {
    //     nh_param->param<int>("axis_angular", pimpl_->axis_angular_map["yaw"], 0);
    //     nh_param->param<double>("scale_angular", pimpl_->scale_angular_map["normal"]["yaw"], 0.5);
    //     nh_param->param<double>("scale_angular_turbo", pimpl_->scale_angular_map["turbo"]["yaw"], pimpl_->scale_angular_map["normal"]["yaw"]);
    // }

    // ROS_INFO_NAMED("TeleopTwistJoy", "Teleop enable button %i.", pimpl_->enable_button);
    // ROS_INFO_COND_NAMED(pimpl_->enable_turbo_button >= 0, "TeleopTwistJoy", "Turbo on button %i.", pimpl_->enable_turbo_button);

    // for (auto it = pimpl_->axis_linear_map.begin(); it != pimpl_->axis_linear_map.end(); ++it) 
    // {
    //     ROS_INFO_NAMED("TeleopTwistJoy", "Linear axis %s on %i at scale %f.",
    //                     it->first.c_str(), it->second,
    //                     pimpl_->scale_linear_map["normal"][it->first]);
    //     ROS_INFO_COND_NAMED(pimpl_->enable_turbo_button >= 0, "TeleopTwistJoy",
    //                         "Turbo for linear axis %s is scale %f.",
    //                         it->first.c_str(),
    //                         pimpl_->scale_linear_map["turbo"][it->first]);
    // }

    // for (auto it = pimpl_->axis_angular_map.begin(); it != pimpl_->axis_angular_map.end(); ++it) {
    //     ROS_INFO_NAMED("TeleopTwistJoy", "Angular axis %s on %i at scale %f.",
    //                     it->first.c_str(), it->second,
    //                     pimpl_->scale_angular_map["normal"][it->first]);
    //     ROS_INFO_COND_NAMED(pimpl_->enable_turbo_button >= 0, "TeleopTwistJoy",
    //                         "Turbo for angular axis %s is scale %f.",
    //                         it->first.c_str(),
    //                         pimpl_->scale_angular_map["turbo"][it->first]);
    // }

    pimpl_->sent_disable_msg = false;
}

double getVal(const sensor_msgs::msg::Joy::ConstPtr& joy_msg,
              const std::map<std::string, int>& axis_map,
              const std::map<std::string, double>& scale_map,
              const std::string& fieldname) 
{
    if (axis_map.find(fieldname) == axis_map.end() ||
        scale_map.find(fieldname) == scale_map.end() ||
        joy_msg->axes.size() <= axis_map.at(fieldname)) 
    {
        return 0.0;
    }

    return joy_msg->axes[axis_map.at(fieldname)] * scale_map.at(fieldname);
}

void TeleopTwistJoy::Impl::sendCmdVelMsg(const sensor_msgs::msg::Joy::SharedPtr joy_msg, const std::string& which_map) 
{
    // Initializes with zeros by default.
    geometry_msgs::msg::Twist cmd_vel_msg;

    cmd_vel_msg.linear.x = getVal(joy_msg, axis_linear_map, scale_linear_map[which_map], "x");
    cmd_vel_msg.linear.y = getVal(joy_msg, axis_linear_map, scale_linear_map[which_map], "y");
    cmd_vel_msg.linear.z = getVal(joy_msg, axis_linear_map, scale_linear_map[which_map], "z");
    cmd_vel_msg.angular.z = getVal(joy_msg, axis_angular_map, scale_angular_map[which_map], "yaw");
    cmd_vel_msg.angular.y = getVal(joy_msg, axis_angular_map, scale_angular_map[which_map], "pitch");
    cmd_vel_msg.angular.x = getVal(joy_msg, axis_angular_map, scale_angular_map[which_map], "roll");

    cmd_vel_pub->publish(cmd_vel_msg);
    std_msgs::msg::UInt8 control_mode_msg;

    if (joy_msg->buttons[safety_button]) {
        control_mode_msg.data = 4;
        control_mode_pub->publish(control_mode_msg);
    } else if (joy_msg->buttons[sit_button]) {
        control_mode_msg.data = 0;
        control_mode_pub->publish(control_mode_msg);
    } else if (joy_msg->buttons[stand_button]) {
        control_mode_msg.data = 1;
        control_mode_pub->publish(control_mode_msg);
    }

    sent_disable_msg = false;
}

void TeleopTwistJoy::Impl::joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg) 
{
    if (enable_turbo_button >= 0 &&
        joy_msg->buttons.size() > enable_turbo_button &&
        joy_msg->buttons[enable_turbo_button]) {
        sendCmdVelMsg(joy_msg, "turbo");
    } else if (true) {
        sendCmdVelMsg(joy_msg, "normal");
    } else {
        // When enable button is released, immediately send a single no-motion
        // command in order to stop the robot.
        if (!sent_disable_msg) {
            // Initializes with zeros by default.
            geometry_msgs::msg::Twist cmd_vel_msg;
            cmd_vel_pub->publish(cmd_vel_msg);
            sent_disable_msg = true;
        }
    }
}

}  // namespace quad_teleop
}  // namespace ros2_quadruped