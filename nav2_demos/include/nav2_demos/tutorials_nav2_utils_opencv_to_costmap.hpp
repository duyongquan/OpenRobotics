#ifndef ROS2_TUTORIALS__TUTORIALS_NAV2_UTILS_OPENCV_TO_COSTMAP_HPP_
#define ROS2_TUTORIALS__TUTORIALS_NAV2_UTILS_OPENCV_TO_COSTMAP_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "grid_map_ros/grid_map_ros.hpp"
#include "cv_bridge/cv_bridge.h"

using namespace std::chrono_literals;

namespace ros2_tutorials
{
namespace nav2
{

class ImageConvertCostmap2D
{
public:
    ImageConvertCostmap2D(rclcpp::Node* node, const std::string & map_topic);
    ~ImageConvertCostmap2D();

    nav_msgs::msg::OccupancyGrid GetMap(const std::string& image_file);

    void PublishMap(const nav_msgs::msg::OccupancyGrid& data);

    void SetTopic(const std::string & name);

     sensor_msgs::msg::Image::SharedPtr ToROS(const cv::Mat& image);

private:
    rclcpp::Node::SharedPtr node_{nullptr};
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_{nullptr};
    std::shared_ptr<grid_map::GridMap> map_{nullptr};
    std::string map_topic_;
    double resolution_;
    double min_height_;
    double max_height_;
};

}  // namespace nav2
}  // namespace ros2_tutorials

#endif  // ROS2_TUTORIALS__TUTORIALS_NAV2_UTILS_OPENCV_TO_COSTMAP_HPP_