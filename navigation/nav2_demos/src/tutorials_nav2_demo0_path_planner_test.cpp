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

        if (start_ok && goal_ok) {
            RCLCPP_ERROR(this->get_logger(), "Start compute A->B path");
            bool success = path_planner_->createPlan(start, goal, path);
            RCLCPP_ERROR(this->get_logger(), "Start compute A->B finished");
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
        std::string pgm = GetMapsPath() + "map.pgm";
        path_planner_->activate();
        path_planner_->loadMap(pgm);
        load_finished_ = true;
    }

    rclcpp::TimerBase::SharedPtr timer_ {nullptr};
    std::shared_ptr<PosesPublisher> pose_publisher_ {nullptr};
    std::shared_ptr<PlannerTester> path_planner_{nullptr};

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
