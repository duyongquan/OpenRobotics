#include "quad_utils/mesh_to_grid_map_converter.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include <pcl/io/vtk_lib_io.h>
#include <pcl_ros/point_cloud.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <grid_map_pcl/GridMapPclConverter.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

namespace ros2_quadruped {
namespace quad_utils {

MeshToGridMapConverter::MeshToGridMapConverter(rclcpp::Node* node)
: node_(node),
  grid_map_resolution_(kDefaultGridMapResolution),
  layer_name_(kDefaultLayerName),
  latch_grid_map_pub_(kDefaultLatchGridMapPub),
  verbose_(kDefaultVerbose),
  frame_id_mesh_loaded_(kDefaultFrameIdMeshLoaded),
  world_name_(kDefaultWorldName) 
{
    // Initial interaction with ROS
    subscribeToTopics();
    advertiseTopics();
    advertiseServices();
    getParametersFromRos();

    std::string package_path = ament_index_cpp::get_package_share_directory("quad_gazebo");
    std::string full_path = package_path + "/worlds/" + world_name_ + "/" + world_name_ + ".ply";
    std::cout << full_path << std::endl;
    bool success = loadMeshFromFile(full_path);
}

void MeshToGridMapConverter::subscribeToTopics() 
{
    mesh_sub_ = node_->create_subscription<pcl_msgs::msg::PolygonMesh>(
        "mesh", 10, std::bind(&MeshToGridMapConverter::meshCallback, this, std::placeholders::_1));
}

void MeshToGridMapConverter::advertiseTopics()
{
    grid_map_pub_ = node_->create_publisher<grid_map_msgs::msg::GridMap>("terrain_map_raw", 1);
}

void MeshToGridMapConverter::advertiseServices()
{
    // save_grid_map_srv_ = node_->create_service<grid_map_msgs::srv::ProcessFile>(
    //   "save_grid_map_to_file", std::bind(&MeshToGridMapConverter::saveGridMapService, this));

    // load_map_service_server_ = node_->create_service<grid_map_msgs::srv::ProcessFile>(
    //   "load_mesh_from_file", std::bind(&MeshToGridMapConverter::loadMeshService, this));
}

void MeshToGridMapConverter::getParametersFromRos()
{
    node_->declare_parameter("grid_map_resolution", grid_map_resolution_);
    node_->declare_parameter("layer_name", layer_name_);
    node_->declare_parameter("latch_grid_map_pub", latch_grid_map_pub_);
    node_->declare_parameter("verbose", verbose_);
    node_->declare_parameter("frame_id_mesh_loaded", frame_id_mesh_loaded_);
    node_->declare_parameter("world", world_name_);

    node_->get_parameter("grid_map_resolution", grid_map_resolution_);
    node_->get_parameter("layer_name", layer_name_);
    node_->get_parameter("latch_grid_map_pub", latch_grid_map_pub_);
    node_->get_parameter("verbose", verbose_);
    node_->get_parameter("frame_id_mesh_loaded", frame_id_mesh_loaded_);
    node_->get_parameter("world", world_name_);
}

bool MeshToGridMapConverter::saveGridMapService(
    const grid_map_msgs::srv::ProcessFile::Request::SharedPtr request,
    const grid_map_msgs::srv::ProcessFile::Response::SharedPtr response)
{
    return true;
}


bool MeshToGridMapConverter::loadMeshService(
    const grid_map_msgs::srv::ProcessFile::Request::SharedPtr request,
    const grid_map_msgs::srv::ProcessFile::Response::SharedPtr response)
{
    return true;
}

bool MeshToGridMapConverter::loadMeshFromFile(const std::string& path_to_mesh_to_load)
{
    return true;
}

void MeshToGridMapConverter::meshCallback(const pcl_msgs::msg::PolygonMesh::SharedPtr mesh_msg) 
{
    if (verbose_) {
        RCLCPP_INFO(node_->get_logger(), "Mesh received, starting conversion.");
    }

    // Converting from message to an object
    pcl::PolygonMesh polygon_mesh;
    pcl_conversions::toPCL(mesh_msg, polygon_mesh);
    meshToGridMap(polygon_mesh, mesh_msg->header.frame_id, mesh_msg.header.stamp.toNSec());
}

// Converts a mesh to grid map and stores the result
bool MeshToGridMapConverter::meshToGridMap(const pcl::PolygonMesh& polygon_mesh,
                    const std::string& mesh_frame_id,
                    const uint64_t& time_stamp_nano_seconds)
{
    return true;
}


// Saves the grid map
bool MeshToGridMapConverter::saveGridMap(const grid_map::GridMap& map,
                    const std::string& path_to_file,
                    const std::string& topic_name)
{
    return true;
}


}  // namespace quad_utils
}  // namespace ros2_quadruped