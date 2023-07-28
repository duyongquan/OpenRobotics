#ifndef ROS2_QUADRUPED__QUAD_GLOBAL_PLANNER__TERRAIN_MAP_PUBLISHER_HPP_
#define ROS2_QUADRUPED__QUAD_GLOBAL_PLANNER__TERRAIN_MAP_PUBLISHER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "grid_map_core/grid_map_core.hpp"
#include "grid_map_ros/grid_map_ros.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <string>
#include <vector>
#include <sstream> //istringstream
#include <iostream> // cout
#include <fstream> // ifstream

namespace ros2_quadruped {
namespace quad_global_planner {


class TerrainMapPublisher : public rclcpp::Node
{
public:
    explicit TerrainMapPublisher();

    /**
     * @brief Creates the map object from scratch
     */
    void createMap();

    /**
     * @brief Loads data from a specified CSV file into a nested std::vector structure
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

    /**
     * @brief Load yaml parameters
     */
    void loadParams();

private:
    /// ROS Subscriber for image data
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_{nullptr};

    /// ROS Publisher for the terrain map
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr terrain_map_pub_{nullptr};

    /// Publish for terrain map
    rclcpp::TimerBase::SharedPtr timer_{nullptr};

    /// terrain map topic
    std::string terrain_map_topic_;

    /// image topic from picture
    std::string image_topic_;

    /// Update rate for sending and receiving data, unused since pubs are called in callbacks
    double update_rate_;

    /// Handle for the map frame
    std::string map_frame_;

    /// GridMap object for terrain data
    grid_map::GridMap terrain_map_;

    /// String of the source of the terrain map data
    std::string map_data_source_;

    /// String of terrain type (if loading from csv)
    std::string terrain_type_;

    /// Bool to flag if the map has been initialized yet
    bool map_initialized_ = false;

    /// Double for map resolution
    double resolution_;

    /// Double for map resolution
    double min_height_;

    /// Double for map resolution
    double max_height_;
};

}  // namespace quad_global_planner
}  // namespace ros2_quadruped

#endif  // ROS2_QUADRUPED__QUAD_GLOBAL_PLANNER__TERRAIN_MAP_PUBLISHER_HPP_