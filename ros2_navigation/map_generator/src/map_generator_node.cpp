
// int
// main(int argc, char** argv)
// {


//   ros::Publisher pcl_pub =
//     nh.advertise<sensor_msgs::PointCloud2>("mock_map", 1);
//   pcl::PointCloud<pcl::PointXYZ> cloud;
//   sensor_msgs::PointCloud2       output;
//   // Fill in the cloud data

//   int seed;

//   int sizeX;
//   int sizeY;
//   int sizeZ;

//   double scale;
//   double update_freq;

//   int type;

//   nh_private.param("seed", seed, 4546);
//   nh_private.param("update_freq", update_freq, 1.0);
//   nh_private.param("resolution", scale, 0.38);
//   nh_private.param("x_length", sizeX, 100);
//   nh_private.param("y_length", sizeY, 100);
//   nh_private.param("z_length", sizeZ, 10);

//   nh_private.param("type", type, 1);

//   scale = 1 / scale;
//   sizeX = sizeX * scale;
//   sizeY = sizeY * scale;
//   sizeZ = sizeZ * scale;

//   mocka::Maps::BasicInfo info;
//   info.nh_private = &nh_private;
//   info.sizeX      = sizeX;
//   info.sizeY      = sizeY;
//   info.sizeZ      = sizeZ;
//   info.seed       = seed;
//   info.scale      = scale;
//   info.output     = &output;
//   info.cloud      = &cloud;

//   mocka::Maps map;
//   map.setInfo(info);
//   map.generate(type);

//   //  optimizeMap(info);

//   //! @note publish loop
//   ros::Rate loop_rate(update_freq);
//   while (ros::ok())
//   {
//     pcl_pub.publish(output);
//     ros::spinOnce();
//     loop_rate.sleep();
//   }
//   return 0;
// }


#include "map_generator/maps.hpp"
#include "map_generator/perlinnoise.hpp"

#include <algorithm>
#include <iostream>
#include <vector>
#include <memory>
#include <thread>
#include <chrono>

#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace map_generator {
namespace {

class MapGenerator: public rclcpp::Node
{
public:
  explicit MapGenerator() : Node("map_generator_node")
  {
    pcl_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("mock_map", rclcpp::SystemDefaultsQoS());
    task_trhead_ = std::make_shared<std::thread>(&MapGenerator::RunTask, this);
  }

  void RunTask()
  {
    int seed;
    int sizeX;
    int sizeY;
    int sizeZ;
    double scale;
    double update_freq;
    int type;

    double width_min;
    double width_max;
    int obstacle_number;
    double connectivity;
    int node_rad;
    int road_rad;
    int num_nodes;
    double complexity;
    double fill;
    int fractal;
    double attenuation;
    double road_width;
    int add_wall_x;
    int add_wall_y;
    int maze_type;

    this->declare_parameter("seed", 4546);
    this->declare_parameter("update_freq", 1.0);
    this->declare_parameter("resolution", 0.38);
    this->declare_parameter("x_length", 100);
    this->declare_parameter("y_length", 100);
    this->declare_parameter("z_length", 10);
    this->declare_parameter("type", 1);

    this->declare_parameter("width_min", 0.6);
    this->declare_parameter("width_max", 1.5);
    this->declare_parameter("obstacle_number", 10);
    this->declare_parameter("connectivity", 0.5);
    this->declare_parameter("node_rad", 3);
    this->declare_parameter("road_rad", 2);
    this->declare_parameter("num_nodes", 10);
    this->declare_parameter("complexity", 0.142857);
    this->declare_parameter("fill", 0.38);
    this->declare_parameter("fractal", 1);
    this->declare_parameter("attenuation", 0.5);
    this->declare_parameter("add_wall_x", 0);
    this->declare_parameter("add_wall_y", 0);
    this->declare_parameter("maze_type", 1);

    seed = this->get_parameter("seed").as_int();
    update_freq = this->get_parameter("update_freq").as_double();
    scale = this->get_parameter("resolution").as_double();
    sizeX = this->get_parameter("x_length").as_int();
    sizeY = this->get_parameter("y_length").as_int();
    sizeZ = this->get_parameter("z_length").as_int();
    type = this->get_parameter("type").as_int();

    width_min = this->get_parameter("width_min").as_double();
    width_max = this->get_parameter("width_max").as_double();
    obstacle_number = this->get_parameter("obstacle_number").as_int();
    connectivity = this->get_parameter("connectivity").as_double();
    node_rad = this->get_parameter("node_rad").as_int();
    road_rad = this->get_parameter("road_rad").as_int();
    num_nodes = this->get_parameter("num_nodes").as_int();
    complexity = this->get_parameter("complexity").as_double();
    fill = this->get_parameter("fill").as_double();
    fractal = this->get_parameter("fractal").as_int();
    attenuation = this->get_parameter("attenuation").as_double();
    add_wall_x = this->get_parameter("add_wall_x").as_int();
    add_wall_y = this->get_parameter("add_wall_y").as_int();
    maze_type = this->get_parameter("maze_type").as_int();

    scale = 1 / scale;
    sizeX = sizeX * scale;
    sizeY = sizeY * scale;
    sizeZ = sizeZ * scale;

    sensor_msgs::msg::PointCloud2  output;
    pcl::PointCloud<pcl::PointXYZ> cloud;

    Maps::BasicInfo info;
    info.sizeX      = sizeX;
    info.sizeY      = sizeY;
    info.sizeZ      = sizeZ;
    info.seed       = seed;
    info.scale      = scale;
    info.output     = &output;
    info.cloud      = &cloud;

    Maps::Config config;
    config.width_min = width_min;
    config.width_max = width_max;
    config.obstacle_number = obstacle_number;
    config.connectivity = connectivity;
    config.node_rad = node_rad;
    config.road_rad = road_rad;
    config.num_nodes = num_nodes;
    config.complexity = complexity;
    config.fill = fill;
    config.fractal = fractal;
    config.attenuation = attenuation;
    config.road_width = road_width;
    config.add_wall_x = add_wall_x;
    config.add_wall_y = add_wall_y;
    config.maze_type = maze_type;

    Maps map;
    map.setInfo(info);
    map.setConfig(config);
    map.generate(type);

    optimizeMap(info);

    while (true) {
      pcl_pub_->publish(output);
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }

private:
  void optimizeMap(map_generator::Maps::BasicInfo& in)
  {
    std::vector<int>* temp = new std::vector<int>;

    pcl::KdTreeFLANN<pcl::PointXYZ>     kdtree;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    cloud->width  = in.cloud->width;
    cloud->height = in.cloud->height;
    cloud->points.resize(cloud->width * cloud->height);

    for (int i = 0; i < cloud->width; i++) {
      cloud->points[i].x = in.cloud->points[i].x;
      cloud->points[i].y = in.cloud->points[i].y;
      cloud->points[i].z = in.cloud->points[i].z;
    }

    kdtree.setInputCloud(cloud);
    double radius = 1.75 / in.scale; // 1.75 is the rounded up value of sqrt(3)

    for (int i = 0; i < cloud->width; i++) {
      std::vector<int>   pointIdxRadiusSearch;
      std::vector<float> pointRadiusSquaredDistance;

      if (kdtree.radiusSearch(cloud->points[i], radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) >= 27)
      {
        temp->push_back(i);
      }
    }
    for (int i = temp->size() - 1; i >= 0; i--) {
      in.cloud->points.erase(in.cloud->points.begin() + temp->at(i)); // erasing the enclosed points
    }
    in.cloud->width -= temp->size();

    pcl::toROSMsg(*in.cloud, *in.output);
    in.output->header.frame_id = "map";
    RCLCPP_INFO(this->get_logger(), "finish: number of points after optimization %d", in.cloud->width);
    delete temp;
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_pub_{nullptr};
  std::shared_ptr<std::thread> task_trhead_{nullptr};
};

}  // namespace
}  // namespace map_generator


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<map_generator::MapGenerator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
