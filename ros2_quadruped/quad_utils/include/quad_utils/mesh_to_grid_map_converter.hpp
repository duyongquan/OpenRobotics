#ifndef ROS2_QUADRUPED__QUAD_SIMULATOR_QUAD_UTILS_MESH_TO_GRID_MAP_CONVERTER_HPP_
#define ROS2_QUADRUPED__QUAD_SIMULATOR_QUAD_UTILS_MESH_TO_GRID_MAP_CONVERTER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "grid_map_core/GridMap.hpp"
#include "grid_map_msgs/msg/grid_map.hpp"
#include "grid_map_msgs/srv/process_file.hpp"
#include "pcl/PolygonMesh.h"
#include "pcl_msgs/msg/polygon_mesh.hpp"
// #include "pcl_ros/point_cloud.h"

#include <string>

namespace ros2_quadruped {
namespace quad_utils {

constexpr double kDefaultGridMapResolution = 0.2;
static const std::string kDefaultLayerName = "elevation";
constexpr bool kDefaultLatchGridMapPub = true;
constexpr bool kDefaultVerbose = false;
static const std::string kDefaultFrameIdMeshLoaded = "map";
static const std::string kDefaultWorldName = "flat";

class MeshToGridMapConverter 
{
public:
    MeshToGridMapConverter(rclcpp::Node* node);

private:
    // Initial interactions with ROS
    void subscribeToTopics();
    void advertiseTopics();
    void advertiseServices();
    void getParametersFromRos();

    // Datacallback
    void meshCallback(const pcl_msgs::msg::PolygonMesh::SharedPtr mesh);

    // Save callback
    bool saveGridMapService(const grid_map_msgs::srv::ProcessFile::Request::SharedPtr request,
                            const grid_map_msgs::srv::ProcessFile::Response::SharedPtr response);

    // Load mesh, service call
    bool loadMeshService(const grid_map_msgs::srv::ProcessFile::Request::SharedPtr request,
                         const grid_map_msgs::srv::ProcessFile::Response::SharedPtr response);

    // Load mesh from file
    bool loadMeshFromFile(const std::string& path_to_mesh_to_load);

    // Converts a mesh to grid map and stores the result
    bool meshToGridMap(const pcl::PolygonMesh& polygon_mesh,
                        const std::string& mesh_frame_id,
                        const uint64_t& time_stamp_nano_seconds);

    // Saves the grid map
    bool saveGridMap(const grid_map::GridMap& map,
                     const std::string& path_to_file,
                     const std::string& topic_name);

    // Node Handles
    rclcpp::Node* node_{nullptr};
    rclcpp::Subscription<pcl_msgs::msg::PolygonMesh>::SharedPtr mesh_sub_{nullptr};
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_pub_{nullptr};
    rclcpp::Service<grid_map_msgs::srv::ProcessFile>::SharedPtr save_grid_map_srv_{nullptr};
    rclcpp::Service<grid_map_msgs::srv::ProcessFile>::SharedPtr load_map_service_server_{nullptr};

    // Last grid map
    std::unique_ptr<grid_map::GridMap> last_grid_map_ptr_;

    // Grid Map Parameters
    double grid_map_resolution_;
    std::string layer_name_;
    std::string world_name_;

    // Control Parameters
    bool latch_grid_map_pub_;
    bool verbose_;

    // Load mesh parameters
    std::string frame_id_mesh_loaded_;
};


}  // namespace quad_utils
}  // namespace ros2_quadruped

#endif  // ROS2_QUADRUPED__QUAD_SIMULATOR_QUAD_UTILS_MESH_TO_GRID_MAP_CONVERTER_HPP_