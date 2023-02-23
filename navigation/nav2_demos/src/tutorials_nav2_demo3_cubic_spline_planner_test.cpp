#include <chrono>
#include <memory>
#include <string>
#include <cmath>
#include <cstdlib>
#include <random>
#include <ctime>
#include <algorithm>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/path.hpp"

#include "spline.h"
#include "nav2_demos/tutorials_nav2_utils_poses_publisher.hpp"

using namespace std::chrono_literals;

namespace ros2_tutorials
{
namespace nav2
{
namespace
{

class CubicSplinePathNode : public rclcpp::Node
{
public:
    CubicSplinePathNode() : Node("cubic_spline_path"), init_point_min_x_(0),init_point_max_x_(10)
    {
        interpolation_number_ = (init_point_max_x_ - init_point_min_x_) * 10;
        cubic_spline_path_publisher_ = std::make_shared<PosesPublisher>(this);
        timer_ = create_wall_timer(
            1000ms, std::bind(&CubicSplinePathNode::HandleTimerCallback, this));
    }

    ~CubicSplinePathNode() {}

private:

    void HandleTimerCallback()
    {
        CreatePoints();
        cubic_spline_path_publisher_->PublishPoses(path_points_);
    }
    
    void InitiPoints()
    {
      std::random_device rd_x;
      std::mt19937 r_eng_x(rd_x());
      std::uniform_real_distribution<double> distribution_x(init_point_min_x_, init_point_max_x_);
      
      for(int i = 0; i < (init_point_max_x_-init_point_min_x_); i++){
        path_points_x_.emplace_back(distribution_x(r_eng_x));
      }
      std::sort(path_points_x_.begin(),path_points_x_.end());

      std::default_random_engine e;
      std::normal_distribution<double> u(0,3);
      e.seed(time(0));
      for (int i = 0; i < (path_points_x_.size()); i++){
          double random = u(e);
          path_points_y_.emplace_back(random);
      }
    }
    
    void InterpolationPoints()
    {
      tk::spline s(path_points_x_, path_points_y_);
      double interval = ((init_point_max_x_ - init_point_min_x_)/(double)interpolation_number_);
      
      for(int i = 0 ; i < interpolation_number_; i++){
        geometry_msgs::msg::Point32 interpolation_point;
        path_points_x_.emplace_back(init_point_min_x_ + interval*(i+1));
        interpolation_point.y = s(init_point_min_x_ + interval*(i+1));
        path_points_y_.emplace_back(interpolation_point.y);
      }
    }

    void CreatePoints()
    {
        path_points_.header.frame_id = "map";
        path_points_.header.stamp = this->get_clock()->now();

        if(!create_points_success_){
          InitiPoints();
          InterpolationPoints();
          create_points_success_ = true;
          
          for (int i = 0; i < path_points_x_.size(); i++) {
              geometry_msgs::msg::Point32 point;
              point.x = path_points_x_[i];
              point.y = path_points_y_[i];
              path_points_.points.push_back(point);
          }
        }
    }

    rclcpp::TimerBase::SharedPtr timer_ {nullptr};
    std::shared_ptr<PosesPublisher> cubic_spline_path_publisher_ {nullptr};
    int init_point_min_x_ {0};
    int init_point_max_x_ {10};
    int interpolation_number_ {100};
    std::vector<double> path_points_x_;
    std::vector<double> path_points_y_;
    sensor_msgs::msg::PointCloud path_points_;
    bool create_points_success_ {false};
};  
}

}  // namespace nav2WW
}  // namespace ros2_tutorials


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ros2_tutorials::nav2::CubicSplinePathNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
