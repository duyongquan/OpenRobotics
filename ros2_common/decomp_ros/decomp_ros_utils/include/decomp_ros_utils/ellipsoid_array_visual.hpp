#ifndef DECOMP_ROS__DECOMP_ROS_UTILS__ELLIPSOID_ARRAY_VISUAL_HPP_
#define DECOMP_ROS__DECOMP_ROS_UTILS__ELLIPSOID_ARRAY_VISUAL_HPP_

#include <decomp_ros_msgs/msg/ellipsoid_array.hpp>
#include <decomp_geometry/ellipsoid.h>
#include <Eigen/Eigenvalues>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include "rviz_rendering/objects/shape.hpp"

namespace decomp_rviz_plugins {
  class EllipsoidArrayVisual {
    public:
      EllipsoidArrayVisual(Ogre::SceneManager *scene_manager, Ogre::SceneNode *parent_node);

      virtual ~EllipsoidArrayVisual();

      void setMessage(const decomp_ros_msgs::msg::EllipsoidArray::ConstPtr &msg);

      void setFramePosition(const Ogre::Vector3 &position);
      void setFrameOrientation(const Ogre::Quaternion &orientation);

      void setColor(float r, float g, float b, float a);

    private:
      std::vector<std::unique_ptr<rviz_rendering::Shape>> objs_;

      Ogre::SceneNode *frame_node_;

      Ogre::SceneManager *scene_manager_;
  };
}
#endif  // DECOMP_ROS__DECOMP_ROS_UTILS__ELLIPSOID_ARRAY_VISUAL_HPP_
