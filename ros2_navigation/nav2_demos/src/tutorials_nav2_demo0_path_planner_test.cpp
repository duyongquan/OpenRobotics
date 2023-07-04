#include <chrono>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/path.hpp"

#include "nav2_demos/tutorials_nav2_utils.hpp"

namespace ros2_tutorials
{
namespace nav2
{
namespace
{

class PathPlanner : public rclcpp::Node
{
public:
    PathPlanner() : Node("path_planner")
    {
        declare_parameter("yaml_filename", rclcpp::PARAMETER_STRING);

        path_planner_ = std::make_shared<PlannerTester>();
        pose_publisher_ = std::make_shared<PosesPublisher>(this);
        timer_ = create_wall_timer(
            2000ms, std::bind(&PathPlanner::HandleTimerCallback, this));
    }

    ~PathPlanner() {}

private:

    void HandleTimerCallback()
    {  
        RunTask();

        geometry_msgs::msg::PoseStamped start;
        geometry_msgs::msg::PoseStamped goal;
        bool start_ok = pose_publisher_->GetStartPoint(start);
        bool goal_ok = pose_publisher_->GetGoalPoint(goal);
        nav_msgs::msg::Path path;

        bool update = CheckGoalUpdate(start, goal);
        if (!update) {
            return;
        }

        if (start_ok && goal_ok) {
            RCLCPP_ERROR(this->get_logger(), "Start compute A->B path");
            bool success = path_planner_->createPlan(start, goal, path);
            if (!success) {
                RCLCPP_ERROR(this->get_logger(), "create path failed");
            } else {
                path_planner_->publishPath(path);
            }
        }
    }

    void RunTask()
    {
        if (load_finished_) {
            return;
        }
        // std::string pgm = GetMapsPath() + "map.pgm";
        path_planner_->activate();

        // Get the name of the YAML file to use
        std::string yaml_filename =  GetMapsPath() + get_parameter("yaml_filename").as_string();
        RCLCPP_INFO(this->get_logger(), "yaml_filename %s",yaml_filename.c_str());
        bool success = path_planner_->loadMapFromYaml(yaml_filename);
        if (!success) {
            RCLCPP_ERROR(this->get_logger(), "Load map %s failed",yaml_filename.c_str() );
            return;
        }
        load_finished_ = true;
    }


    bool CheckGoalUpdate(const geometry_msgs::msg::PoseStamped& start, 
        const geometry_msgs::msg::PoseStamped & goal)
    {
        double dx = start.pose.position.x - goal.pose.position.x;
        double dy = start.pose.position.y - goal.pose.position.y;

        auto dist = std::hypotf(dx, dy);
        if ( std::abs( dist - distance_) > 0.05) {
            distance_ = dist;
            return true;
        }
        return false;
    }

    rclcpp::TimerBase::SharedPtr timer_ {nullptr};
    std::shared_ptr<PosesPublisher> pose_publisher_ {nullptr};
    std::shared_ptr<PlannerTester> path_planner_{nullptr};

    double distance_{0.0f};
    bool load_finished_{false};
}; 

}  // namespace
}  // namespace nav2
}  // namespace ros2_tutorials

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ros2_tutorials::nav2::PathPlanner>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
