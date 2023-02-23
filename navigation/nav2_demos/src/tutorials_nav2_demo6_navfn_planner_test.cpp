#include <chrono>
#include <memory>
#include <string>
#include <cmath>
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/path.hpp"

#include "nav2_navfn_planner/navfn_planner.hpp"
#include "nav2_demos/tutorials_nav2_utils_poses_publisher.hpp"
#include "nav2_demos/tutorials_nav2_utils_opencv_to_costmap.hpp"
#include "nav2_demos/tutorials_nav2_utils_dataset_loader.hpp"

using namespace std::chrono_literals;

namespace ros2_tutorials
{
namespace nav2
{
namespace
{

class NavFnPlanner : public rclcpp::Node
{
public:
    NavFnPlanner() : Node("navfn_planner")
    {
        rclcpp::QoS map_qos(10);
        costmap_ = std::make_shared<ImageConvertCostmap2D>(this, "map");
        // Setup the global costmap
        costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
            "global_costmap", std::string{get_namespace()}, "global_costmap");
        costmap_ros_->on_configure(rclcpp_lifecycle::State());
        costmap_ros_->activate();

        // Launch a thread to run the costmap node
        costmap_thread_ = std::make_unique<nav2_util::NodeThread>(costmap_ros_);

        start_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "initialpose", map_qos,
                std::bind(&NavFnPlanner::HandleStartPointCallback, this, std::placeholders::_1));

        goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "goal_pose", map_qos,
                std::bind(&NavFnPlanner::HandleGoalPointCallback, this, std::placeholders::_1));

        pose_publisher_ = std::make_shared<PosesPublisher>(this);
        planner_ = std::make_shared<nav2_navfn_planner::NavfnPlanner>();
        timer_ = create_wall_timer(
            1000ms, std::bind(&NavFnPlanner::HandleTimerCallback, this));
    }

    ~NavFnPlanner() {}

private:
    void HandleTimerCallback()
    {
        // Load map
        LoadMap();

        // Make path and publish
        bool success = CreatePath();
        if (!success) {
            RCLCPP_ERROR(this->get_logger(), "Create plath error");
        } else {
            RCLCPP_INFO(this->get_logger(), "Create plath success.");
        }
    }

    void HandleStartPointCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        if (msg == nullptr) {
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Receive start initial pose");
        recevie_start_pose_ = true;

        // set pose
        start_.header.frame_id = "map";
        start_.header.stamp = this->get_clock()->now();
        start_.pose = msg->pose.pose;
    }

    void HandleGoalPointCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (msg == nullptr) {
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Receive goal target pose");

        // set pose
        goal_.header.frame_id = "map";
        goal_.header.stamp = this->get_clock()->now();
        goal_.pose = msg->pose;
        recevie_goal_pose_ = true;
    }

    bool CreatePath()
    {
        if (recevie_start_pose_ && recevie_goal_pose_) {
            auto node = shared_from_this();
            tf_ = costmap_ros_->getTfBuffer();
            // planner_->configure(node, "NavFn", tf_, costmap_ros_);
            planner_->activate();
            path_ = planner_->createPlan(start_, goal_);
            pose_publisher_->PublishPath(path_);
            ResetFlags();
            return true;
        }

        pose_publisher_->PublishPath(path_);
        return false;
    }   

    void LoadMap()
    {
        if (load_map_finished_) {
            return;
        }

        std::string map_name = GetMapsPath() + "map01.yaml";
        bool success = costmap_->GetOccupancyGridMapFromYaml(map_name, map_);
        if (!success) {
            RCLCPP_ERROR(this->get_logger(), "Load OccupancyGrid error");
            return;
        }
        costmap_->PublishMap(map_);
        load_map_finished_ = true;

        costmap_ros_->start();
    }

    void ResetFlags()
    {
        recevie_start_pose_ = false;
        recevie_goal_pose_ = false;
    }

    // TF buffer
    std::shared_ptr<tf2_ros::Buffer> tf_;

    rclcpp::TimerBase::SharedPtr timer_{nullptr};
    std::shared_ptr<PosesPublisher> pose_publisher_ {nullptr};
    std::shared_ptr<ImageConvertCostmap2D> costmap_{nullptr};
    std::unique_ptr<nav2_util::NodeThread> costmap_thread_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_{nullptr};
    nav_msgs::msg::OccupancyGrid map_;

    std::shared_ptr<nav2_navfn_planner::NavfnPlanner> planner_{nullptr};
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr start_sub_{nullptr};
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_{nullptr};

    geometry_msgs::msg::PoseStamped start_;
    geometry_msgs::msg::PoseStamped goal_; 
    nav_msgs::msg::Path path_;
    bool recevie_start_pose_{false};
    bool recevie_goal_pose_{false};
    bool load_map_finished_{false};
};


}  // namespace
}  // namespace nav2
}  // namespace ros2_tutorials


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ros2_tutorials::nav2::NavFnPlanner>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
