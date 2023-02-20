#include <chrono>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/path.hpp"

#include "nav2_demos/tutorials_nav2_utils.hpp"

using namespace std::chrono_literals;

namespace ros2_tutorials
{
namespace nav2
{
namespace
{

class AStarPlanner : public rclcpp::Node
{
public:
    AStarPlanner() : Node("a_star_planner")
    {
        a_star_palnner_ = std::make_shared<PlannerTester>();
        pose_publisher_ = std::make_shared<PosesPublisher>(this);
        timer_ = create_wall_timer(
            2000ms, std::bind(&AStarPlanner::HandleTimerCallback, this));
    }

    ~AStarPlanner() {}

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
            // bool success = a_star_palnner_->createPlan(start, goal, path);

            bool success = a_star_palnner_->defaultPlannerRandomTests(100, 0.1);
            if (!success) {
                RCLCPP_ERROR(this->get_logger(), "create path failed");
            } else {
                a_star_palnner_->publishPath(path);
            }
        }
    }

    void RunTask()
    {
        if (load_finished_) {
            return;
        }
        std::string pgm = GetMapsPath() + "map.pgm";
        a_star_palnner_->activate();
        a_star_palnner_->loadMap(pgm);
        load_finished_ = true;
    }

    rclcpp::TimerBase::SharedPtr timer_ {nullptr};
    std::shared_ptr<PosesPublisher> pose_publisher_ {nullptr};
    std::shared_ptr<PlannerTester> a_star_palnner_{nullptr};

    bool load_finished_{false};
};  
}

}  // namespace nav2WW
}  // namespace ros2_tutorials


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ros2_tutorials::nav2::AStarPlanner>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
