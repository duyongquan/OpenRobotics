#include "gridmap_ros/tutorials_gridmap_demo1_opencv.hpp"

namespace ros2_tutorials
{
namespace gridmap_ros
{

OpencvNode::OpencvNode()
: Node("opencv_demo")
{
    RCLCPP_INFO(get_logger(), "Creating opencv_demo node.");
}

OpencvNode::~OpencvNode()
{

}

}  // namespace gridmap_ros
}  // namespace ros2_tutorials