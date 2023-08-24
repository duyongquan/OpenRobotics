
#ifndef DECOMP_ROS__DECOMP_ROS_UTILS__ELLIPSOID_ARRAY_DISPLAY_HPP_
#define DECOMP_ROS__DECOMP_ROS_UTILS__ELLIPSOID_ARRAY_DISPLAY_HPP_

#include "decomp_ros_utils/ellipsoid_array_visual.hpp"

#include <decomp_ros_msgs/msg/ellipsoid_array.hpp>
#include "rviz_common/message_filter_display.hpp"
#include "rviz_common/visualization_manager.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "frame_manager.hpp"
#include "rviz_common/load_resource.hpp"

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <QObject>
#include <QtWidgets>
#include <QBasicTimer>

namespace decomp_rviz_plugins {

class EllipsoidArrayVisual;

class EllipsoidArrayDisplay
    : public rviz::MessageFilterDisplay<decomp_ros_msgs::EllipsoidArray> {
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
  void processMessage(const decomp_ros_msgs::EllipsoidArray::ConstPtr &msg);

  std::shared_ptr<EllipsoidArrayVisual> visual_;

  rviz::ColorProperty *color_property_;
  rviz::FloatProperty *alpha_property_;
};
}

#endif // DECOMP_ROS__DECOMP_ROS_UTILS__ELLIPSOID_ARRAY_DISPLAY_HPP_