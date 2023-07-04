#include "nav2_demos/tutorials_nav2_utils_controller_test.hpp"

#include <string>
#include <random>
#include <tuple>
#include <utility>
#include <vector>
#include <memory>
#include <iostream>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <unistd.h>

#include "geometry_msgs/msg/twist.hpp"
#include "nav2_map_server/map_mode.hpp"
#include "nav2_map_server/map_io.hpp"
#include "nav2_msgs/msg/costmap_meta_data.hpp"

using namespace std::chrono_literals;
using namespace std::chrono;  // NOLINT
using nav2_util::Costmap;
using nav2_util::TestCostmap;

namespace ros2_tutorials
{
namespace nav2
{

NavControllerTester::NavControllerTester(): nav2_controller::ControllerServer()
{

}

void NavControllerTester::onCleanup(const rclcpp_lifecycle::State & state)
{
    on_cleanup(state);
}

void NavControllerTester::onActivate(const rclcpp_lifecycle::State & state)
{
    on_activate(state);
}

void NavControllerTester::onDeactivate(const rclcpp_lifecycle::State & state)
{
    on_deactivate(state);
}

void NavControllerTester::onConfigure(const rclcpp_lifecycle::State & state)
{
    on_configure(state);
}

void NavControllerTester::SetPlannerPath(const nav_msgs::msg::Path & path)
{
    setPlannerPath(path);
}

void NavControllerTester::ComputeControl()
{
    computeControl();
}

void NavControllerTester::Activate()
{
    auto state = rclcpp_lifecycle::State();
    onConfigure(state);
    rclcpp::Rate r(1);
    r.sleep();
    onActivate(state);
}

}  // namespace nav2
}  // namespace ros2_tutorials