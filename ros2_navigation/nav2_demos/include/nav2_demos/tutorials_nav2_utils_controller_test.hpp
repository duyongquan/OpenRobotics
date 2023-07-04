#ifndef ROS2_TUTORIALS__TUTORIALS_NAV2_UTILS_CONTROLLER_TEST_HPP_
#define ROS2_TUTORIALS__TUTORIALS_NAV2_UTILS_CONTROLLER_TEST_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav2_msgs/srv/get_costmap.hpp"
#include "nav2_util/costmap.hpp"
#include "nav2_util/node_thread.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "nav2_controller/nav2_controller.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/marker.hpp"

using namespace std::chrono_literals;

namespace ros2_tutorials
{
namespace nav2
{

class NavControllerTester : public nav2_controller::ControllerServer
{
public:
    NavControllerTester();

    void onCleanup(const rclcpp_lifecycle::State & state);

    void onActivate(const rclcpp_lifecycle::State & state);

    void onDeactivate(const rclcpp_lifecycle::State & state);
  
    void onConfigure(const rclcpp_lifecycle::State & state);

    void SetPlannerPath(const nav_msgs::msg::Path & path);

    void ComputeControl();

    void Activate();
private:

    std::shared_ptr<rclcpp_action::Client<nav2_msgs::action::FollowPath>> action_client_;
};

}  // namespace nav2
}  // namespace ros2_tutorials

#endif  // ROS2_TUTORIALS__TUTORIALS_NAV2_UTILS_CONTROLLER_TEST_HPP_