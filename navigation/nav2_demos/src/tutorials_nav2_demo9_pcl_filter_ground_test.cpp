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

class PCLFilterGround : public rclcpp::Node
{
public:
    PCLFilterGround() : Node("filter_ground")
    {
        timer_ = create_wall_timer(
            1000ms, std::bind(&PCLFilterGround::HandleTimerCallback, this));
    }

    ~PCLFilterGround() {}

private:
    void HandleTimerCallback()
    {

    }

 

    rclcpp::TimerBase::SharedPtr timer_{nullptr};
};

}  // namespace
}  // namespace nav2
}  // namespace ros2_tutorials


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ros2_tutorials::nav2::PCLFilterGround>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
