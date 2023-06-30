#ifndef ROS2_TUTORIALS__GRIDMAP_ROS_DEMO1_OPENCV_HPP_
#define ROS2_TUTORIALS__GRIDMAP_ROS_DEMO1_OPENCV_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "grid_map_ros/grid_map_ros.hpp"
#include "grid_map_cv/grid_map_cv.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui/highgui.hpp"


namespace ros2_tutorials
{
namespace gridmap_ros
{

class OpencvNode : public rclcpp::Node
{
public:
    OpencvNode();
    ~OpencvNode();

private:
    bool use_ransparency_ {false};
};

}  // namespace gridmap_ros
}  // namespace ros2_tutorials

#endif  // ROS2_TUTORIALS__GRIDMAP_ROS_DEMO1_OPENCV_HPP_