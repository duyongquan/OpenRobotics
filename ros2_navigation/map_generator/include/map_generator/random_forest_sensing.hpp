#ifndef MAP_GENERATOR_PERLINNOISE_HPP_
#define MAP_GENERATOR_PERLINNOISE_HPP_

#include <iostream>
#include <cmath>
#include <random>
#include <chrono>
#include <memory>
#include <string>

#include <Eigen/Eigen>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"


namespace map_generator {
class RandomForestMap : public rclcpp::Node
{
public:
    RandomForestMap();
    ~RandomForestMap();

    void RandomMapGenerate();

    void RandomMapGenerateCylinder();

private:
    void HandleClickCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    void HandleOdometryCallbck(const nav_msgs::msg::Odometry::SharedPtr odom);

    void PublishSensedPoints();

    void HandleTimerCallback();

    void LoadParameters();

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_map_pub_ {nullptr};
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr all_map_pub_ {nullptr};
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr click_map_pub_ {nullptr};
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_{nullptr};

    rclcpp::TimerBase::SharedPtr map_timer_ {nullptr};

    // pcl::search::KdTree<pcl::PointXYZ> kdtreeLocalMap;
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtreeLocalMap;
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    std::random_device rd;
    std::default_random_engine eng {0};
    // std::default_random_engine eng(rd); 
    std::uniform_real_distribution<double> rand_x;
    std::uniform_real_distribution<double> rand_y;
    std::uniform_real_distribution<double> rand_w;
    std::uniform_real_distribution<double> rand_h;
    std::uniform_real_distribution<double> rand_inf;

    std::vector<double> _state;

    int _obs_num;
    double _x_size, _y_size, _z_size;
    double _x_l, _x_h, _y_l, _y_h, _w_l, _w_h, _h_l, _h_h;
    double _z_limit, _sensing_range, _resolution, _sense_rate, _init_x, _init_y;
    double _min_dist;

    bool _map_ok = false;
    bool _has_odom = false;

    int circle_num_;
    double radius_l_, radius_h_, z_l_, z_h_;
    double theta_;
    std::uniform_real_distribution<double> rand_radius_;
    std::uniform_real_distribution<double> rand_radius2_;
    std::uniform_real_distribution<double> rand_theta_;
    std::uniform_real_distribution<double> rand_z_;

    sensor_msgs::msg::PointCloud2 globalMap_pcd;
    pcl::PointCloud<pcl::PointXYZ> cloudMap;

    sensor_msgs::msg::PointCloud2 localMap_pcd;
    pcl::PointCloud<pcl::PointXYZ> clicked_cloud_;

    bool map_generate_finished_{false};
};

}  // namespace map_generator 
#endif // MAP_GENERATOR_PERLINNOISE_HPP_
