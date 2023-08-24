#ifndef DECOMP_ROS__DECOMP_ROS_UTILS__POLYHEDRON_ARRAY_DISPLAY_HPP_
#define DECOMP_ROS__DECOMP_ROS_UTILS__POLYHEDRON_ARRAY_DISPLAY_HPP_

#include "decomp_ros_msgs/msg/polyhedron_array.hpp"

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include "rviz_common/load_resource.hpp"
#include "rviz_common/frame_manager.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/int_property.hpp"
#include "rviz_common/properties/enum_property.hpp
#include "rviz_common/visualization_manager.hpp"
#include "rviz_common/message_filter_display.hpp"

#include "decomp_ros_utils/mesh_visual.hpp"
#include "decomp_ros_utils/bound_visual.hpp"
#include "decomp_ros_utils/vector_visual.hpp"
#include "decomp_ros_utils/data_ros_utils.hpp"
#include "decomp_geometry/geometric_utils.h"

namespace decomp_rviz_plugins {
class PolyhedronArrayDisplay
    : public rviz::MessageFilterDisplay<decomp_ros_msgs::PolyhedronArray> {
  Q_OBJECT
public:
  PolyhedronArrayDisplay();
  virtual ~PolyhedronArrayDisplay();

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
  void processMessage(const decomp_ros_msgs::PolyhedronArray::ConstPtr &msg);
  void visualizeMessage(int state);
  void visualizeMesh();
  void visualizeBound();
  void visualizeVs();

  std::shared_ptr<MeshVisual> visual_mesh_;
  std::shared_ptr<BoundVisual> visual_bound_;
  std::shared_ptr<VectorVisual> visual_vector_;

  rviz::ColorProperty *mesh_color_property_;
  rviz::ColorProperty *bound_color_property_;
  rviz::ColorProperty *vs_color_property_;
  rviz::FloatProperty *alpha_property_;
  rviz::FloatProperty *scale_property_;
  rviz::FloatProperty *vs_scale_property_;
  rviz::EnumProperty *state_property_;

  Ogre::Vector3 position_;
  Ogre::Quaternion orientation_;

  vec_E<vec_Vec3f> vertices_;
  vec_E<std::pair<Vec3f, Vec3f>> vs_;
};

}
#endif  // DECOMP_ROS__DECOMP_ROS_UTILS__POLYHEDRON_ARRAY_DISPLAY_HPP_