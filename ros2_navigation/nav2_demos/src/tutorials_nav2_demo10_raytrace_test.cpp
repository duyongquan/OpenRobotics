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

class RayTrace : public rclcpp::Node
{
public:
    RayTrace() : Node("ray_trace")
    {
        timer_ = create_wall_timer(
            1000ms, std::bind(&RayTrace::HandleTimerCallback, this));
    }

    ~RayTrace() {}


    enum class FillShape { Circle, Rectangle};

private:
    void HandleTimerCallback()
    {

    } 

    void PublishCostmap();
 
    void CreateCostmapData(int width, int height, double resolution, 
        double x0, double y0, double x1, double y1, const FillShape& shape);

    rclcpp::TimerBase::SharedPtr timer_{nullptr};
};

}  // namespace
}  // namespace nav2
}  // namespace ros2_tutorials


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ros2_tutorials::nav2::RayTrace>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
