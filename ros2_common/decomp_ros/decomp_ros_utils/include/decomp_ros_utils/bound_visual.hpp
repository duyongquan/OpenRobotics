#ifndef DECOMP_ROS__DECOMP_ROS_UTILS__BOUND_VISUAL_HPP_
#define DECOMP_ROS__DECOMP_ROS_UTILS__BOUND_VISUAL_HPP_

#include <decomp_basis/data_type.h>

#include "rviz_rendering/objects/billboard_line.hpp"
#include "rviz_rendering/objects/shape.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/message_filter_display.hpp"

namespace decomp_rviz_plugins {
class BoundVisual {
public:
  BoundVisual(Ogre::SceneManager *scene_manager, Ogre::SceneNode *parent_node);
  ~BoundVisual();

  void setMessage(const vec_E<vec_Vec3f> &bds);
  void setFramePosition(const Ogre::Vector3 &position);
  void setFrameOrientation(const Ogre::Quaternion &orientation);

  void setColor(float r, float g, float b, float a);
  void setScale(float s);

private:
  std::vector<std::unique_ptr<rviz_rendering::BillboardLine>> objs_;

  Ogre::SceneNode *frame_node_;

  Ogre::SceneManager *scene_manager_;
};
}

#endif  // DECOMP_ROS__DECOMP_ROS_UTILS__BOUND_VISUAL_HPP_
