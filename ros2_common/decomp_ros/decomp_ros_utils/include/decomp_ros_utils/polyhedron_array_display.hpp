#ifndef DECOMP_ROS__DECOMP_ROS_UTILS__POLYHEDRON_ARRAY_DISPLAY_HPP_
#define DECOMP_ROS__DECOMP_ROS_UTILS__POLYHEDRON_ARRAY_DISPLAY_HPP_

#include "decomp_ros_msgs/msg/polyhedron_array.hpp"

#include "rviz_common/load_resource.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/int_property.hpp"
#include "rviz_common/properties/enum_property.hpp"
#include "rviz_common/message_filter_display.hpp"


#include "decomp_ros_utils/data_ros_utils.hpp"
#include "decomp_geometry/geometric_utils.h"

#include <QObject>
#include <QtWidgets>

#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreSceneNode.h>
#include <Ogre.h>
#include <OgreQuaternion.h>

namespace decomp_rviz_plugins {

class MeshVisual;
class BoundVisual;
class VectorVisual;

class PolyhedronArrayDisplay
    : public rviz_common::MessageFilterDisplay<decomp_ros_msgs::msg::PolyhedronArray> 
{
  Q_OBJECT

public:
  PolyhedronArrayDisplay();
  ~PolyhedronArrayDisplay();

protected:
  virtual void onInitialize();

  virtual void reset();

private Q_SLOTS:
  void updateMeshColorAndAlpha();
  void updateBoundColorAndAlpha();
  void updateVsColorAndAlpha();
  void updateState();
  void updateScale();
  void updateVsScale();

private:
  void processMessage(const decomp_ros_msgs::msg::PolyhedronArray::SharedPtr msg);
  void visualizeMessage(int state);
  void visualizeMesh();
  void visualizeBound();
  void visualizeVs();

  std::shared_ptr<MeshVisual> visual_mesh_;
  std::shared_ptr<BoundVisual> visual_bound_;
  std::shared_ptr<VectorVisual> visual_vector_;

  rviz_common::properties::ColorProperty *mesh_color_property_;
  rviz_common::properties::ColorProperty *bound_color_property_;
  rviz_common::properties::ColorProperty *vs_color_property_;
  rviz_common::properties::FloatProperty *alpha_property_;
  rviz_common::properties::FloatProperty *scale_property_;
  rviz_common::properties::FloatProperty *vs_scale_property_;
  rviz_common::properties::EnumProperty *state_property_;

  Ogre::Vector3 position_;
  Ogre::Quaternion orientation_;

  vec_E<vec_Vec3f> vertices_;
  vec_E<std::pair<Vec3f, Vec3f>> vs_;
};

}
#endif  // DECOMP_ROS__DECOMP_ROS_UTILS__POLYHEDRON_ARRAY_DISPLAY_HPP_