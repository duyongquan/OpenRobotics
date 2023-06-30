
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


namespace ros2_tutorials
{
namespace params
{

// bt_navigator
class BTNavigatorNode : public rclcpp::Node
{
public:
    BTNavigatorNode() : Node("bt_navigator")
    {

    }

private:


};


// controller_server
class ControllerServerNode : public rclcpp::Node
{
public:
    ControllerServerNode() : Node("controller_server")
    {

    }

private:


};


// global_costmap
class GlobalCostmapNode : public rclcpp::Node
{
public:
    GlobalCostmapNode() : Node("global_costmap")
    {

    }

private:


};

// planner_server
class PlannerServerNode : public rclcpp::Node
{
public:
    PlannerServerNode() : Node("planner_server")
    {

    }


private:

};

// recoveries_server
class RecoveriesServerNode : public rclcpp::Node
{
public:
    RecoveriesServerNode() : Node("recoveries_server")
    {

    }

private:

};

}  // namespace params
}  // namespace ros2_tutorials


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    // You MUST use the MultiThreadedExecutor to use, well, multiple threads
    rclcpp::executors::MultiThreadedExecutor executor;
    auto bt_navigator_node = std::make_shared<ros2_tutorials::params::BTNavigatorNode>();
    auto controller_server_node = std::make_shared<ros2_tutorials::params::ControllerServerNode>();
    auto global_costmap_node = std::make_shared<ros2_tutorials::params::GlobalCostmapNode>();
    auto planner_server_node = std::make_shared<ros2_tutorials::params::PlannerServerNode>();
    auto recoveries_server_node = std::make_shared<ros2_tutorials::params::RecoveriesServerNode>();

    // Executor add node 
    executor.add_node(bt_navigator_node);
    executor.add_node(controller_server_node);
    executor.add_node(global_costmap_node);
    executor.add_node(planner_server_node);
    executor.add_node(recoveries_server_node);

    // Spin here
    executor.spin();
    rclcpp::shutdown();
    return 0;
}