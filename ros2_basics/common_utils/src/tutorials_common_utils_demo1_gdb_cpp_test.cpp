#include <chrono>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"


namespace ros2_tutorials
{
namespace common_utils
{
namespace
{


class GDBTester : public rclcpp::Node
{
public:
    GDBTester() : Node("gdb_tester")
    {
        RunTask();
    }

    ~GDBTester() {}

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
  auto node = std::make_shared<ros2_tutorials::common_utils::GDBTester>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
