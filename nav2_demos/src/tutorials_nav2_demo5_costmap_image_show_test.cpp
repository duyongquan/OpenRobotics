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

class ImageCostmap2D : public rclcpp::Node
{
public:
    ImageCostmap2D() : Node("bezier_path")
    {
        costmap_2d_ = std::make_shared<ImageConvertCostmap2D>(this, "global_map");

        timer_ = create_wall_timer(
            1000ms, std::bind(&ImageCostmap2D::HandleTimerCallback, this));
    }

    ~ImageCostmap2D() {}

private:
    void HandleTimerCallback()
    {
        // ShowImage();
        auto map = costmap_2d_->GetMap("map03.pgm");
        map.header.frame_id = "map";
        map.header.stamp = this->get_clock()->now();
        costmap_2d_->PublishMap(map);
    }

    void ShowImage()
    {
        LoadImage("map02.png", true);
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
  auto node = std::make_shared<ros2_tutorials::nav2::ImageCostmap2D>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
