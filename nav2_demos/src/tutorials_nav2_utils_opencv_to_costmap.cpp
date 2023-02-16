#include "nav2_demos/tutorials_nav2_utils_opencv_to_costmap.hpp"
#include "nav2_demos/tutorials_nav2_utils_dataset_loader.hpp"

namespace ros2_tutorials
{
namespace nav2
{

ImageConvertCostmap2D::ImageConvertCostmap2D(rclcpp::Node* node, const std::string & map_topic)
: map_topic_{map_topic},
  resolution_{0.05},
  min_height_{0.0},
  max_height_{1.0},
  node_{node}
{
    map_ = std::make_shared<grid_map::GridMap>(grid_map::GridMap({"elevation"}));
    
    auto custom_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    costmap_pub_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
        map_topic,
        custom_qos);
}

ImageConvertCostmap2D::~ImageConvertCostmap2D()
{
}

nav_msgs::msg::OccupancyGrid ImageConvertCostmap2D::GetMap(const std::string& image_file)
{
    // Load opencv format image
    cv::Mat map_image = LoadImage(image_file);
    auto image = ToROS(map_image);
    grid_map::GridMapRosConverter::initializeFromImage(*image, resolution_, *map_);

    RCLCPP_INFO(
      node_->get_logger(),
      "Initialized map with size %f x %f m (%i x %i cells).", map_->getLength().x(),
      map_->getLength().y(), map_->getSize()(0), map_->getSize()(1));
    grid_map::GridMapRosConverter::addLayerFromImage(*image, "elevation", *map_, 0.0, 254);

    // Convert grid map to ros
    nav_msgs::msg::OccupancyGrid occupancy_grid;
    grid_map::GridMapRosConverter::toOccupancyGrid(*map_, "elevation", 0.0, 100.0, occupancy_grid);
    return occupancy_grid;
}

void ImageConvertCostmap2D::PublishMap(const nav_msgs::msg::OccupancyGrid& data)
{
    costmap_pub_->publish(data);
}

void ImageConvertCostmap2D::SetTopic(const std::string & name)
{
    map_topic_ = name;
}

sensor_msgs::msg::Image::SharedPtr ImageConvertCostmap2D::ToROS(const cv::Mat& image)
{
    return cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
}

}  // namespace nav2
}  // namespace ros2_tutorials
