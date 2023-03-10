#include "quad_gazebo/contact_plugin.hpp"
#include "quad_msgs/msg/contact_mode.hpp"

#include <builtin_interfaces/msg/time.hpp>
#include <gazebo/plugins/ContactPlugin.hh>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#ifdef IGN_PROFILER_ENABLE
#include <ignition/common/Profiler.hh>
#endif

#include <iostream>
#include <memory>
#include <string>

namespace gazebo_plugins
{

class ContactPluginSensorPrivate
{
public:
    /// Node for ros communication
    gazebo_ros::Node::SharedPtr ros_node_;
    /// Publish for contact mode message
    rclcpp::Publisher<quad_msgs::msg::ContactMode>::SharedPtr pub_;
    /// Contact message modified each update
    quad_msgs::msg::ContactMode::SharedPtr msg_;
    /// Contact sensor this plugin is attached to
    gazebo::sensors::ContactSensorPtr sensor_;
    /// Event triggered when sensor updates
    gazebo::event::ConnectionPtr sensor_update_event_;
    /// Publish latest imu data to ROS
    void OnUpdate();
};

ContactPluginSensor::ContactPluginSensor()
: impl_(std::make_unique<ContactPluginSensorPrivate>())
{
}

ContactPluginSensor::~ContactPluginSensor()
{
    impl_->ros_node_.reset();
}

void ContactPluginSensor::Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
    impl_->ros_node_ = gazebo_ros::Node::Get(_sdf, _sensor);
    // Get QoS profiles
    const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();

    impl_->sensor_ = std::dynamic_pointer_cast<gazebo::sensors::ContactSensor>(_sensor);
    if (!impl_->sensor_) {
        RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Parent is not a contact sensor. Exiting.");
        impl_->ros_node_.reset();
        return;
    }

    impl_->pub_ = impl_->ros_node_->create_publisher<quad_msgs::msg::ContactMode>(
        "gazebo/toe_forces", qos.get_publisher_qos("gazebo/toe_forces", rclcpp::SensorDataQoS().reliable()));

    RCLCPP_INFO(
        impl_->ros_node_->get_logger(), "Publishing contact states to [%s]",
        impl_->pub_->get_topic_name());

    // Create message to be reused
    auto msg = std::make_shared<quad_msgs::msg::ContactMode>();

    // Get frame for message
    msg->header.frame_id = gazebo_ros::SensorFrameID(*_sensor, *_sdf);

    impl_->msg_ = msg;
    impl_->sensor_update_event_ = impl_->sensor_->ConnectUpdated(
        std::bind(&ContactPluginSensorPrivate::OnUpdate, impl_.get()));
    impl_->sensor_->SetActive(true);
}

void ContactPluginSensorPrivate::OnUpdate()
{
    gazebo::msgs::Contacts contacts;
    contacts = sensor_->Contacts();

    // Initialize outgoing messages with zeros (send zero contact force if no
    // contact detected)
    quad_msgs::msg::ContactMode contact_msg;
    contact_msg.leg_contacts.resize(4);

    std::string toe_collision_names[4] = {
        "toe0_collision", 
        "toe1_collision",
        "toe2_collision",
        "toe3_collision"};

    // Assume no contact
    for (unsigned int i = 0; i < 4; ++i) {
        contact_msg.leg_contacts.at(i).contact_prob = 0.0;
        contact_msg.leg_contacts.at(i).contact_state = false;
        contact_msg.leg_contacts.at(i).contact_forces.x = 0;
        contact_msg.leg_contacts.at(i).contact_forces.y = 0;
        contact_msg.leg_contacts.at(i).contact_forces.z = 0;
    }

    // Populate messages with contact forces
    for (unsigned int i = 0; i < contacts.contact_size(); ++i) {
        std::string str = contacts.contact(i).collision1();
        for (unsigned int j = 0; j < 4; ++j) {
            std::string toe_string = toe_collision_names[j];
            std::size_t found_toe = str.find(toe_string);
            if (found_toe != std::string::npos) {
                contact_msg.leg_contacts.at(j).contact_prob = 1.0;
                contact_msg.leg_contacts.at(j).contact_state = true;
                for (unsigned int k = 0; k < contacts.contact(i).position_size(); ++k) {
                    contact_msg.leg_contacts.at(j).contact_forces.x -= contacts.contact(i).wrench(k).body_1_wrench().force().x();
                    contact_msg.leg_contacts.at(j).contact_forces.y -= contacts.contact(i).wrench(k).body_1_wrench().force().y();
                    contact_msg.leg_contacts.at(j).contact_forces.z -= contacts.contact(i).wrench(k).body_1_wrench().force().z();
                }
            }
        }
    }

  this->pub_->publish(contact_msg);
}

GZ_REGISTER_SENSOR_PLUGIN(ContactPluginSensor)
}  // namespace gazebo_plugins


