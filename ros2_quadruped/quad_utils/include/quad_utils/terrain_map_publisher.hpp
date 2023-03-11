#ifndef ROS2_QUADRUPED__QUAD_SIMULATOR_QUAD_UTILS_TERRAIN_MAP_PUBLISHER_HPP_
#define ROS2_QUADRUPED__QUAD_SIMULATOR_QUAD_UTILS_TERRAIN_MAP_PUBLISHER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "quad_utils/mesh_to_grid_map_converter.hpp"

#include <fstream>  // ifstream
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <iostream>  // cout
#include <sstream>   // istringstream
#include <string>
#include <vector>
#include <memory>

namespace ros2_quadruped {
namespace quad_utils {

struct Obstacle 
{
    double x;
    double y;
    double height;
    double radius;
};
struct Step 
{
    double x;
    double height;
};

class TerrainMapPublisher : public rclcpp::Node
{
public:
    TerrainMapPublisher();
    ~TerrainMapPublisher();

private:
    std::shared_ptr<MeshToGridMapConverter> map_converter_{nullptr};

    /**
     * @brief Updates the terrain_map_publisher parameters
     */
    void updateParams();

    /**
     * @brief Creates the map object from scratch
     */
    void createMap();

    /**
     * @brief Updates the map object with params
     */
    void updateMap();

    /**
     * @brief Loads data from a specified CSV file into a nested std::vector
     * structure
     * @param[in] filename Path to the CSV file
     * @return Data from the CSV in vector structure
     */
    std::vector<std::vector<double> > loadCSV(std::string filename);

    /**
     * @brief Loads data into the map object from a CSV
     */
    void loadMapFromCSV();

    /**
     * @brief Loads data into the map object from an image topic
     * @param[in] msg ROS image message
     */
    void loadMapFromImage(const sensor_msgs::msg::Image::SharedPtr msg);

    /**
     * @brief Publishes map data to the terrain_map topic
     */
    void publishMap();

    void HandleTimerCallback();

 private:

    rclcpp::TimerBase::SharedPtr timer_;

    /// ROS Subscriber for image data
    // ros::Subscriber image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

    /// ROS Publisher for the terrain map
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr terrain_map_pub_;

    /// Update rate for sending and receiving data, unused since pubs are called
    /// in callbacks
    double update_rate_;

    /// Handle for the map frame
    std::string map_frame_;

    /// grid_map::GridMap object for terrain data
    grid_map::GridMap terrain_map_;

    /// String for the terrain file name
    std::string terrain_type_;

    /// string of the source of the terrain map data
    std::string map_data_source_;

    /// bool to flag if the map has been initialized yet
    bool map_initialized_ = false;

    /// double for map resolution
    double resolution_;

    /// double for map resolution
    double min_height_;

    /// double for map resolution
    double max_height_;

    /// Obstacle object
    Obstacle obstacle_;

    /// Step 1 object
    Step step1_;

    /// Step 2 object
    Step step2_;

    bool load_map_finished_{false};
    bool is_map_type_image_{false};
};

}  // namespace quad_utils
}  // namespace ros2_quadruped

#endif  // ROS2_QUADRUPED__QUAD_SIMULATOR_QUAD_UTILS_TERRAIN_MAP_PUBLISHER_HPP_