#include "map_generator/random_forest_sensing.hpp"

using namespace std::chrono_literals;

namespace map_generator {

RandomForestMap::RandomForestMap() : Node("random_map_sensing")
{
    local_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/map_generator/local_cloud", rclcpp::SystemDefaultsQoS());
    all_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/map_generator/global_cloud", rclcpp::SystemDefaultsQoS());
    click_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/pcl_render_node/local_map", rclcpp::SystemDefaultsQoS());
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odometry", 10, 
            std::bind(&RandomForestMap::HandleOdometryCallbck, this, std::placeholders::_1));

    LoadParameters();

    // timer
    map_timer_ = this->create_wall_timer(1000ms, 
        std::bind(&RandomForestMap::HandleTimerCallback, this));

}

RandomForestMap::~RandomForestMap()
{
}

void RandomForestMap::RandomMapGenerate()
{
    pcl::PointXYZ pt_random;

    rand_x = std::uniform_real_distribution<double>(_x_l, _x_h);
    rand_y = std::uniform_real_distribution<double>(_y_l, _y_h);
    rand_w = std::uniform_real_distribution<double>(_w_l, _w_h);
    rand_h = std::uniform_real_distribution<double>(_h_l, _h_h);

    rand_radius_ = std::uniform_real_distribution<double>(radius_l_, radius_h_);
    rand_radius2_ = std::uniform_real_distribution<double>(radius_l_, 1.2);
    rand_theta_ = std::uniform_real_distribution<double>(-theta_, theta_);
    rand_z_ = std::uniform_real_distribution<double>(z_l_, z_h_);

    // generate polar obs
    for (int i = 0; i < _obs_num; i++) {
        double x, y, w, h;
        x = rand_x(eng);
        y = rand_y(eng);
        w = rand_w(eng);

        if (sqrt(pow(x - _init_x, 2) + pow(y - _init_y, 2)) < 2.0) {
            i--;
            continue;
        }

        if (sqrt(pow(x - 19.0, 2) + pow(y - 0.0, 2)) < 2.0) {
            i--;
            continue;
        }

        x = floor(x / _resolution) * _resolution + _resolution / 2.0;
        y = floor(y / _resolution) * _resolution + _resolution / 2.0;

        int widNum = ceil(w / _resolution);

        for (int r = -widNum / 2.0; r < widNum / 2.0; r++) {
            for (int s = -widNum / 2.0; s < widNum / 2.0; s++) {
                h = rand_h(eng);
                int heiNum = ceil(h / _resolution);
                for (int t = -20; t < heiNum; t++) {
                pt_random.x = x + (r + 0.5) * _resolution + 1e-2;
                pt_random.y = y + (s + 0.5) * _resolution + 1e-2;
                pt_random.z = (t + 0.5) * _resolution + 1e-2;
                cloudMap.points.push_back(pt_random);
                }
            }
        }
    }

    // generate circle obs
    for (int i = 0; i < circle_num_; ++i) {
        double x, y, z;
        x = rand_x(eng);
        y = rand_y(eng);
        z = rand_z_(eng);

        if (sqrt(pow(x - _init_x, 2) + pow(y - _init_y, 2)) < 2.0) {
            i--;
            continue;
        }

        if (sqrt(pow(x - 19.0, 2) + pow(y - 0.0, 2)) < 2.0) {
            i--;
            continue;
        }

        x = floor(x / _resolution) * _resolution + _resolution / 2.0;
        y = floor(y / _resolution) * _resolution + _resolution / 2.0;
        z = floor(z / _resolution) * _resolution + _resolution / 2.0;

        Eigen::Vector3d translate(x, y, z);

        double theta = rand_theta_(eng);
        Eigen::Matrix3d rotate;
        rotate << cos(theta), -sin(theta), 0.0, sin(theta), cos(theta), 0.0, 0, 0, 1;

        double radius1 = rand_radius_(eng);
        double radius2 = rand_radius2_(eng);

        // draw a circle centered at (x,y,z)
        Eigen::Vector3d cpt;
        for (double angle = 0.0; angle < 6.282; angle += _resolution / 2) 
        {
            cpt(0) = 0.0;
            cpt(1) = radius1 * cos(angle);
            cpt(2) = radius2 * sin(angle);

            // inflate
            Eigen::Vector3d cpt_if;
            for (int ifx = -0; ifx <= 0; ++ifx) {
                for (int ify = -0; ify <= 0; ++ify) {
                    for (int ifz = -0; ifz <= 0; ++ifz) {
                        cpt_if = cpt + Eigen::Vector3d(ifx * _resolution, ify * _resolution,
                                                    ifz * _resolution);
                        cpt_if = rotate * cpt_if + Eigen::Vector3d(x, y, z);
                        pt_random.x = cpt_if(0);
                        pt_random.y = cpt_if(1);
                        pt_random.z = cpt_if(2);
                        cloudMap.push_back(pt_random);
                    }
                }
            }
        }
    }

    cloudMap.width = cloudMap.points.size();
    cloudMap.height = 1;
    cloudMap.is_dense = true;

    RCLCPP_INFO(this->get_logger(), "Finished generate random map.");

    kdtreeLocalMap.setInputCloud(cloudMap.makeShared());

    _map_ok = true;
}

void RandomForestMap::RandomMapGenerateCylinder()
{
    pcl::PointXYZ pt_random;

    std::vector<Eigen::Vector2d> obs_position;

    rand_x = std::uniform_real_distribution<double>(_x_l, _x_h);
    rand_y = std::uniform_real_distribution<double>(_y_l, _y_h);
    rand_w = std::uniform_real_distribution<double>(_w_l, _w_h);
    rand_h = std::uniform_real_distribution<double>(_h_l, _h_h);
    rand_inf = std::uniform_real_distribution<double>(0.5, 1.5);

    rand_radius_ = std::uniform_real_distribution<double>(radius_l_, radius_h_);
    rand_radius2_ = std::uniform_real_distribution<double>(radius_l_, 1.2);
    rand_theta_ = std::uniform_real_distribution<double>(-theta_, theta_);
    rand_z_ = std::uniform_real_distribution<double>(z_l_, z_h_);

    // generate polar obs
    for (int i = 0; i < _obs_num; i++) {
        double x, y, w, h, inf;
        x = rand_x(eng);
        y = rand_y(eng);
        w = rand_w(eng);
        inf = rand_inf(eng);

        if (std::sqrt(pow(x - _init_x, 2) + pow(y - _init_y, 2)) < 2.0) {
            i--;
            continue;
        }

        if (std::sqrt(pow(x - 19.0, 2) + pow(y - 0.0, 2)) < 2.0) {
            i--;
            continue;
        }
        
        bool flag_continue = false;
        for ( auto p : obs_position )
        if ( (Eigen::Vector2d(x,y) - p).norm() < _min_dist /*metres*/ )
        {
            i--;
            flag_continue = true;
            break;
        }
        if ( flag_continue ) continue;

        obs_position.push_back( Eigen::Vector2d(x,y) );
        

        x = std::floor(x / _resolution) * _resolution + _resolution / 2.0;
        y = std::floor(y / _resolution) * _resolution + _resolution / 2.0;

        int widNum = ceil((w*inf) / _resolution);
        double radius = (w*inf) / 2;

        for (int r = -widNum / 2.0; r < widNum / 2.0; r++)
            for (int s = -widNum / 2.0; s < widNum / 2.0; s++) {
                h = rand_h(eng);
                int heiNum = ceil(h / _resolution);
                for (int t = -30; t < heiNum; t++) {
                    double temp_x = x + (r + 0.5) * _resolution + 1e-2;
                    double temp_y = y + (s + 0.5) * _resolution + 1e-2;
                    double temp_z = (t + 0.5) * _resolution + 1e-2;
                    if ( (Eigen::Vector2d(temp_x,temp_y) - Eigen::Vector2d(x,y)).norm() <= radius )
                    {
                        pt_random.x = temp_x;
                        pt_random.y = temp_y;
                        pt_random.z = temp_z;
                        cloudMap.points.push_back(pt_random);
                    }
                }
            }
    }

    // generate circle obs
    for (int i = 0; i < circle_num_; ++i) {
        double x, y, z;
        x = rand_x(eng);
        y = rand_y(eng);
        z = rand_z_(eng);

        if (std::sqrt(pow(x - _init_x, 2) + pow(y - _init_y, 2)) < 2.0) {
            i--;
            continue;
        }

        if (std::sqrt(pow(x - 19.0, 2) + pow(y - 0.0, 2)) < 2.0) {
            i--;
            continue;
        }

        x = floor(x / _resolution) * _resolution + _resolution / 2.0;
        y = floor(y / _resolution) * _resolution + _resolution / 2.0;
        z = floor(z / _resolution) * _resolution + _resolution / 2.0;

        Eigen::Vector3d translate(x, y, z);

        double theta = rand_theta_(eng);
        Eigen::Matrix3d rotate;
        rotate << cos(theta), -sin(theta), 0.0, sin(theta), cos(theta), 0.0, 0, 0, 1;

        double radius1 = rand_radius_(eng);
        double radius2 = rand_radius2_(eng);

        // draw a circle centered at (x,y,z)
        Eigen::Vector3d cpt;
        for (double angle = 0.0; angle < 6.282; angle += _resolution / 2) {
        cpt(0) = 0.0;
        cpt(1) = radius1 * cos(angle);
        cpt(2) = radius2 * sin(angle);

        // inflate
        Eigen::Vector3d cpt_if;
        for (int ifx = -0; ifx <= 0; ++ifx)
            for (int ify = -0; ify <= 0; ++ify)
            for (int ifz = -0; ifz <= 0; ++ifz) {
                cpt_if = cpt + Eigen::Vector3d(ifx * _resolution, ify * _resolution,
                                            ifz * _resolution);
                cpt_if = rotate * cpt_if + Eigen::Vector3d(x, y, z);
                pt_random.x = cpt_if(0);
                pt_random.y = cpt_if(1);
                pt_random.z = cpt_if(2);
                cloudMap.push_back(pt_random);
            }
        }
    }

    // generate floor 
    // pcl::PointXYZ pt;
    // pt.z = 0.1;
    // for ( pt.x = _x_l; pt.x <= _x_h; pt.x += _resolution )
    //   for ( pt.y = _y_l; pt.y <= _y_h; pt.y += _resolution )
    //   {
    //     cloudMap.push_back(pt);
    //   }

    cloudMap.width = cloudMap.points.size();
    cloudMap.height = 1;
    cloudMap.is_dense = true;

    RCLCPP_INFO(this->get_logger(), "Finished generate random map.");

    kdtreeLocalMap.setInputCloud(cloudMap.makeShared());

    _map_ok = true;
}

void RandomForestMap::HandleClickCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    double x = msg->pose.position.x;
    double y = msg->pose.position.y;
    double w = rand_w(eng);
    double h;
    pcl::PointXYZ pt_random;

    x = floor(x / _resolution) * _resolution + _resolution / 2.0;
    y = floor(y / _resolution) * _resolution + _resolution / 2.0;

    int widNum = ceil(w / _resolution);

    for (int r = -widNum / 2.0; r < widNum / 2.0; r++) {
        for (int s = -widNum / 2.0; s < widNum / 2.0; s++) {
            h = rand_h(eng);
            int heiNum = ceil(h / _resolution);
            for (int t = -1; t < heiNum; t++) {
                pt_random.x = x + (r + 0.5) * _resolution + 1e-2;
                pt_random.y = y + (s + 0.5) * _resolution + 1e-2;
                pt_random.z = (t + 0.5) * _resolution + 1e-2;
                clicked_cloud_.points.push_back(pt_random);
                cloudMap.points.push_back(pt_random);
            }
        }
    }

    clicked_cloud_.width = clicked_cloud_.points.size();
    clicked_cloud_.height = 1;
    clicked_cloud_.is_dense = true;

    pcl::toROSMsg(clicked_cloud_, localMap_pcd);
    localMap_pcd.header.frame_id = "world";
    click_map_pub_->publish(localMap_pcd);

    cloudMap.width = cloudMap.points.size();
}

void RandomForestMap::HandleOdometryCallbck(const nav_msgs::msg::Odometry::SharedPtr odom)
{
    if (odom->child_frame_id == "X" || odom->child_frame_id == "O")
        return;
    _has_odom = true;

    _state = {odom->pose.pose.position.x,
              odom->pose.pose.position.y,
              odom->pose.pose.position.z,
              odom->twist.twist.linear.x,
              odom->twist.twist.linear.y,
              odom->twist.twist.linear.z,
              0.0,
              0.0,
              0.0};
}

void RandomForestMap::PublishSensedPoints()
{
    // if (i < 10) {
    pcl::toROSMsg(cloudMap, globalMap_pcd);
    globalMap_pcd.header.frame_id = "world";
    all_map_pub_->publish(globalMap_pcd);
    // }

    return;

    /* ---------- only publish points around current position ---------- */
    if (!_map_ok || !_has_odom) return;

    pcl::PointCloud<pcl::PointXYZ> localMap;

    pcl::PointXYZ searchPoint(_state[0], _state[1], _state[2]);
    pointIdxRadiusSearch.clear();
    pointRadiusSquaredDistance.clear();

    pcl::PointXYZ pt;

    if (std::isnan(searchPoint.x) || std::isnan(searchPoint.y) || std::isnan(searchPoint.z))
        return;

    if (kdtreeLocalMap.radiusSearch(searchPoint, _sensing_range,
                                    pointIdxRadiusSearch,
                                    pointRadiusSquaredDistance) > 0) 
    {
        for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i) {
            pt = cloudMap.points[pointIdxRadiusSearch[i]];
            localMap.points.push_back(pt);
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "[Map server] No obstacles .");
        return;
    }

    localMap.width = localMap.points.size();
    localMap.height = 1;
    localMap.is_dense = true;

    pcl::toROSMsg(localMap, localMap_pcd);
    localMap_pcd.header.frame_id = "world";
    local_map_pub_->publish(localMap_pcd);
}

void RandomForestMap::HandleTimerCallback()
{
    if (!map_generate_finished_) {
        // RandomMapGenerate();
        RandomMapGenerateCylinder();
        map_generate_finished_ = true;
    }
    
    PublishSensedPoints();
}

void RandomForestMap::LoadParameters()
{
    this->declare_parameter("init_state_x", 0.0);
    this->declare_parameter("init_state_y", 0.0);
    this->declare_parameter("x_size", 50.0);
    this->declare_parameter("y_size", 50.0);
    this->declare_parameter("z_size", 5.0);
    this->declare_parameter("obs_num", 30);
    this->declare_parameter("resolution", 0.1);
    this->declare_parameter("circle_num", 30);
    this->declare_parameter("lower_rad", 0.3);
    this->declare_parameter("upper_rad", 0.8);
    this->declare_parameter("lower_hei", 3.0);
    this->declare_parameter("upper_hei", 7.0);
    this->declare_parameter("radius_l", 7.0);
    this->declare_parameter("radius_h", 7.0);
    this->declare_parameter("z_l", 7.0);
    this->declare_parameter("z_h", 7.0);
    this->declare_parameter("theta", 7.0);
    this->declare_parameter("radius", 10.0);
    this->declare_parameter("min_distance", 1.0);


    _init_x = this->get_parameter("init_state_x").as_double();
    _init_y = this->get_parameter("init_state_y").as_double();
    _x_size = this->get_parameter("x_size").as_double();
    _y_size = this->get_parameter("y_size").as_double();
    _z_size = this->get_parameter("z_size").as_double();
    _obs_num = this->get_parameter("obs_num").as_int();
    _resolution = this->get_parameter("resolution").as_double();
    circle_num_ = this->get_parameter("circle_num").as_int();
    _w_l = this->get_parameter("lower_rad").as_double();
    _w_h = this->get_parameter("upper_rad").as_double();
    _h_l = this->get_parameter("lower_hei").as_double();
    _h_h = this->get_parameter("upper_hei").as_double();
    radius_l_ = this->get_parameter("radius_l").as_double();
    radius_h_ = this->get_parameter("radius_h").as_double();
    z_l_ = this->get_parameter("z_l").as_double();
    z_h_ = this->get_parameter("z_h").as_double();
    theta_ = this->get_parameter("theta").as_double();
    _sense_rate = this->get_parameter("radius").as_double();
    _min_dist = this->get_parameter("min_distance").as_double();

    _sensing_range = _sense_rate;
    _x_l = -_x_size / 2.0;
    _x_h = +_x_size / 2.0;

    _y_l = -_y_size / 2.0;
    _y_h = +_y_size / 2.0;

    _obs_num = std::min(_obs_num, (int)_x_size * 10);
    _z_limit = _z_size;
}

} // namespace map_generator 