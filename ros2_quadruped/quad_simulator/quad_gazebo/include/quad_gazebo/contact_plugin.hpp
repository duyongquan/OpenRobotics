
#ifndef ROS2_QUADRUPED__QUAD_SIMULATOR_QUAD_GAZEBO_CONTACT_PLUGIN_HPP_
#define ROS2_QUADRUPED__QUAD_SIMULATOR_QUAD_GAZEBO_CONTACT_PLUGIN_HPP_

// #include "geometry_msgs/msgs/vector3.hpp"
// #include "quad_msgs/msg/ContactMode.hpp"

#include <gazebo/common/Plugin.hh>


namespace gazebo_plugins {

class ContactPluginSensorPrivate;

class ContactPluginSensor : public gazebo::SensorPlugin
{
public:
  /// Constructor.
  ContactPluginSensor();
  /// Destructor.
  virtual ~ContactPluginSensor();

  // Documentation Inherited
  void Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

private:
  /// Private data pointer
  std::unique_ptr<ContactPluginSensorPrivate> impl_;
};

}

#endif  // ROS2_QUADRUPED__QUAD_SIMULATOR_QUAD_GAZEBO_CONTACT_PLUGIN_HPP_