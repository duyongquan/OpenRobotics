

#include <geometry_msgs/msg/quaternion.hpp>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

namespace pedsim {

geometry_msgs::msg::Quaternion angleToQuaternion(const double theta);

geometry_msgs::msg::Quaternion rpyToQuaternion(const double roll, const double pitch,
                                          const double yaw);

geometry_msgs::msg::Quaternion toQuaternionMsg(const Eigen::Quaternionf& quaternion);

geometry_msgs::msg::Quaternion poseFrom2DVelocity(const double vx, const double vy);

std::vector<std::pair<float, float>> LineObstacleToCells(const float x1,
                                                         const float y1,
                                                         const float x2,
                                                         const float y2);

}  // namespace pedsim
