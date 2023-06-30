#include <chrono>
#include <memory>
#include <string>
#include <cmath>
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/path.hpp"

#include "nav2_demos/tutorials_nav2_utils_opencv_to_costmap.hpp"
#include "nav2_demos/tutorials_nav2_utils_dataset_loader.hpp"

using namespace std::chrono_literals;

namespace ros2_tutorials
{
namespace nav2
{
namespace
{

class Costmap2DPublisher : public rclcpp::Node
{
public:
    Costmap2DPublisher() : Node("costmap")
    {
        costmap_2d_ = std::make_shared<ImageConvertCostmap2D>(this, "global_map");
        timer_ = create_wall_timer(
            1000ms, std::bind(&Costmap2DPublisher::HandleTimerCallback, this));
    }

    ~Costmap2DPublisher() {}

private:
    void HandleTimerCallback()
    {
        PublishCostmap(100, 20);
    }

 
    void PublishCostmap(const int height, const int width) 
    {
        nav_msgs::msg::OccupancyGrid map;
        map.header.frame_id = "map";
        map.header.stamp = this->get_clock()->now();

        map.info.height = height;
        map.info.width = width;
        map.info.resolution = 0.05;

        
        for (int i = 0; i < height * width; i++) {
            map.data.push_back(98);
        }

        costmap_2d_->PublishMap(map);
    }

    rclcpp::TimerBase::SharedPtr timer_{nullptr};
    std::shared_ptr<ImageConvertCostmap2D> costmap_2d_{nullptr};
};

}  // namespace
}  // namespace nav2
}  // namespace ros2_tutorials


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ros2_tutorials::nav2::Costmap2DPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
