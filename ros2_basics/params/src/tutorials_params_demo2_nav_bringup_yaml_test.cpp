
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav2_util/node_utils.hpp"

using nav2_util::declare_parameter_if_not_declared;

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
        std::vector<std::string> plugin_libs;
        declare_parameter("plugin_lib_names", plugin_libs);
        declare_parameter("transform_tolerance", rclcpp::ParameterValue(0.1));
        declare_parameter("global_frame", std::string("map"));
        declare_parameter("robot_base_frame", std::string("base_link"));
        declare_parameter("odom_topic", std::string("odom"));

        global_frame_ = get_parameter("global_frame").as_string();
        robot_frame_ = get_parameter("robot_base_frame").as_string();
        transform_tolerance_ = get_parameter("transform_tolerance").as_double();
        odom_topic_ = get_parameter("odom_topic").as_string();
        plugin_lib_names_ = get_parameter("plugin_lib_names").as_string_array();

        RCLCPP_INFO(this->get_logger(), "[########################### bt_navigator ###########################]"); 
        PrintParamters();
    }

    void PrintParamters()
    {
        RCLCPP_INFO(this->get_logger(), "global_frame_ : %s", global_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "robot_frame_ : %s", robot_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "transform_tolerance_ : %lf", transform_tolerance_);
        RCLCPP_INFO(this->get_logger(), "odom_topic_ : %s", odom_topic_.c_str());

        // plugin_lib_names 
        for (auto lib : plugin_lib_names_) {
            RCLCPP_INFO(this->get_logger(), "plugin_lib_names  : %s", lib.c_str());
        }
    }

private:

    std::string robot_frame_;
    std::string global_frame_;
    double transform_tolerance_;
    std::string odom_topic_;
    std::vector<std::string>  plugin_lib_names_;
};


// controller_server
class ControllerServerNode : public rclcpp::Node
{
public:
    ControllerServerNode() : Node("controller_server")
    {
        RCLCPP_INFO(get_logger(), "getting goal checker plugins..");
        declare_parameter("goal_checker_plugins", goal_checker_ids_);
        declare_parameter("controller_plugins", controller_ids_);

        get_parameter("goal_checker_plugins", goal_checker_ids_);
        get_parameter("controller_plugins", controller_ids_);

        // FollowPath
        declare_parameter_if_not_declared(this, "FollowPath.critics", rclcpp::PARAMETER_STRING_ARRAY);
        get_parameter("FollowPath.critics", critic_names_);

        RCLCPP_INFO(this->get_logger(), "[########################### controller_server ###########################]"); 
        PrintParamters();
    }

    void PrintParamters()
    {
        // plugin_lib_names 
        for (auto goal_checker_id : goal_checker_ids_) {
            RCLCPP_INFO(this->get_logger(), "goal_checker_id  : %s", goal_checker_id.c_str());
        }

        // plugin_lib_names 
        for (auto controller_id : controller_ids_) {
            RCLCPP_INFO(this->get_logger(), "controller_id : %s", controller_id.c_str());
        }

        // FollowPath 
        for (auto critic_name : critic_names_) {
            RCLCPP_INFO(this->get_logger(), "critic_name : %s", critic_name.c_str());
        }
    }

private:
    std::vector<std::string> goal_checker_ids_;
    std::vector<std::string> controller_ids_;
    std::vector<std::string> critic_names_;
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