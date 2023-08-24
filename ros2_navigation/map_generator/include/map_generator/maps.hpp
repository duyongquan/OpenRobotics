#ifndef MAP_GENERATOR__MAPS_HPP_
#define MAP_GENERATOR__MAPS_HPP_

#include "rclcpp/rclcpp.hpp"

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace map_generator {

class Maps 
{
public:
  typedef struct BasicInfo 
  {
    // ros::NodeHandle *nh_private;
    int sizeX;
    int sizeY;
    int sizeZ;
    int seed;
    double scale;
    sensor_msgs::msg::PointCloud2 *output;
    pcl::PointCloud<pcl::PointXYZ> *cloud;
  } BasicInfo;

  typedef struct Config 
  {
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
  } Config;

  BasicInfo getInfo() const;
  void setInfo(const BasicInfo &value);

  Config getConfig() const;
  void setConfig(const Config &value);

public:
  Maps();

public:
  void generate(int type);

private:
  BasicInfo info;
  Config config;

private:
  void pcl2ros();

  void perlin3D();
  void maze2D();
  void randomMapGenerate();
  void Maze3DGen();
  void recursiveDivision(int xl, int xh, int yl, int yh, Eigen::MatrixXi &maze);
  void recursizeDivisionMaze(Eigen::MatrixXi &maze);
  void optimizeMap();
};

class MazePoint 
{
private:
  pcl::PointXYZ point;
  double dist1;
  double dist2;
  int point1;
  int point2;
  bool isdoor;

public:
  pcl::PointXYZ getPoint();
  int getPoint1();
  int getPoint2();
  double getDist1();
  double getDist2();
  void setPoint(pcl::PointXYZ p);
  void setPoint1(int p);
  void setPoint2(int p);
  void setDist1(double set);
  void setDist2(double set);
};

} // namespace map_generator

#endif // MAP_GENERATOR__MAPS_HPP_
