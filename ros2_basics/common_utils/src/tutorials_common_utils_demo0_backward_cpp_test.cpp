#include <chrono>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include <backward_ros/backward.hpp>
#define BACKWARD_HAS_DW 1

namespace backward {
    backward::SignalHandling sh;
}

namespace ros2_tutorials
{
namespace common_utils
{
namespace
{


class BackwardTester : public rclcpp::Node
{
public:
    BackwardTester() : Node("backward_tester")
    {
        RunTask();
    }

    ~BackwardTester() {}

private:

    void RunTask()
    {
        char *c = "hello world";
        c[1] = 'H';
    }
}; 



}  // namespace
}  // namespace common_utils
}  // namespace ros2_tutorials

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ros2_tutorials::common_utils::BackwardTester>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
