
#ifndef DECOMP_ROS__DECOMP_ROS_UTILS__ELLIPSOID_ARRAY_DISPLAY_HPP_
#define DECOMP_ROS__DECOMP_ROS_UTILS__ELLIPSOID_ARRAY_DISPLAY_HPP_

#include "decomp_ros_utils/ellipsoid_array_visual.hpp"

#include <decomp_ros_msgs/msg/ellipsoid_array.hpp>
#include "rviz_common/message_filter_display.hpp"
#include "rviz_common/visualization_manager.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/load_resource.hpp"

#include <QObject>
#include <QtWidgets>
#include <QBasicTimer>

#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreSceneNode.h>
#include <Ogre.h>
#include <OgreQuaternion.h>

namespace decomp_rviz_plugins {

class EllipsoidArrayVisual;

class EllipsoidArrayDisplay
    : public rviz_common::MessageFilterDisplay<decomp_ros_msgs::msg::EllipsoidArray> {
  Q_OBJECT
public:
  EllipsoidArrayDisplay();
  ~EllipsoidArrayDisplay();

protected:
  void onInitialize();

  void reset();

private Q_SLOTS:
  void updateColorAndAlpha();

private:
  void processMessage(const decomp_ros_msgs::msg::EllipsoidArray::SharedPtr msg);

  std::shared_ptr<EllipsoidArrayVisual> visual_;

  rviz_common::properties::ColorProperty *color_property_;
  rviz_common::properties::FloatProperty *alpha_property_;
};
}

#endif // DECOMP_ROS__DECOMP_ROS_UTILS__ELLIPSOID_ARRAY_DISPLAY_HPP_