
#include <cmath>

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
    // costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("global_costmap");
    // costmap_ros_->on_configure(rclcpp_lifecycle::State());

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

bool ImageConvertCostmap2D::GetOccupancyGridMapFromImage(
    const std::string& image_file,  nav_msgs::msg::OccupancyGrid& map)
{
    // cv::Mat map_image = LoadImage(image_file);
    // int sizex = map_image.cols;
    // int sizey = map_image.rows;

    // // costmap2d
    // double resolution = 0.05;
    // double origin_x = 0.0;
    // double origin_y = 0.0;


    // costmap_ = costmap_ros_->getCostmap();
    // // costmap_ = std::make_shared<nav2_costmap_2d::Costmap2D>(sizex, sizey, resolution, origin_x, origin_y);
    // for(int r = 0; r < sizey; r++){
    //   for(int c = 0; c < sizex; c++ ) {
    //     // ??sizey-r-1 caution: costmap's origin is at left bottom ,
    //     // while opencv's pic's origin is at left-top.
    //     auto data = map_image.at<uchar>(r, c);
    //     costmap_->setCost(c, sizey - r - 1, data);
    //   }
    // }

    // // OccupancyGrid map grid size
    // unsigned int len = costmap_->getSizeInCellsX() * costmap_->getSizeInCellsY();

    // RCLCPP_INFO(node_->get_logger(), "SizeX * SizeY(%d * %d)",
    //     costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());
    // // OccupancyGrid map
    // map.header.frame_id = "map";
    // map.header.stamp = node_->get_clock()->now();

    // // Same as header stamp as we do not load the map.
    // map.info.map_load_time = map.header.stamp;
    // map.info.resolution = resolution;
    // map.info.width = costmap_->getSizeInCellsX();
    // map.info.height = costmap_->getSizeInCellsY();
    // map.info.origin.position.x = origin_x;
    // map.info.origin.position.y = origin_y;
    // map.info.origin.position.z = 0.0;
    // map.info.origin.orientation.x = 0.0;
    // map.info.origin.orientation.y = 0.0;
    // map.info.origin.orientation.z = 0.0;
    // map.info.origin.orientation.w = 1.0;
    // map.data.resize(len);
   
    // // Occupancy probabilities are in the range [0,100]. Unknown is -1.
    // const float cellMin = 0;
    // const float cellMax = 100;
    // const float cellRange = cellMax - cellMin;

    // for (unsigned int i = 0; i < len; i++) {
    //     auto data = costmap_->getCost(i);
    //     float value =  data / 255.0;
    //     if (isnan(value)) {
    //         value = -1;
    //     } else {
    //         value = cellMin + std::min(std::max(0.0f, value), 1.0f) * cellRange;
    //     }
    //     map.data.push_back(value);
    // }
    return true;
}

bool ImageConvertCostmap2D::GetOccupancyGridMapFromYaml(
    const std::string& yaml_file,  nav_msgs::msg::OccupancyGrid& map)
{
    switch (nav2_map_server::loadMapFromYaml(yaml_file, map)) {
        case nav2_map_server::MAP_DOES_NOT_EXIST:
            RCLCPP_ERROR(node_->get_logger(), "map not exist");
            return false;
        case nav2_map_server::INVALID_MAP_METADATA:
            RCLCPP_ERROR(node_->get_logger(), "map metadata is invalid");
            return false;
        case nav2_map_server::INVALID_MAP_DATA:
            RCLCPP_ERROR(node_->get_logger(), "map data is invalid");
            return false;
        case nav2_map_server::LOAD_MAP_SUCCESS:
            // Correcting msg_ header when it belongs to spiecific node
            RCLCPP_INFO(node_->get_logger(), "map load is success");
            map.info.map_load_time = node_->get_clock()->now();
            map.header.frame_id = "map";
            map.header.stamp = node_->get_clock()->now();
    }
    return true;
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

// nav2_costmap_2d::Costmap2D* ImageConvertCostmap2D::GetCostmap2D()
// {
//     return costmap_;
// }

}  // namespace nav2
}  // namespace ros2_tutorials
