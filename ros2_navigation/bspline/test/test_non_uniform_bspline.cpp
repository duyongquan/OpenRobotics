
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_tools/ros_visualization_tools.hpp"
#include "bspline/non_uniform_bspline.hpp"

using namespace std::chrono_literals;

namespace bspline
{
namespace
{

class NonUniformBsplineNode : public rclcpp::Node
{
public:
    NonUniformBsplineNode() : Node("non_uniform_bspline")
    {
        points_markers_ = std::make_shared<visualization_tools::RosVizTools>(this, "control_points");

        // timer
        timer_ = create_wall_timer(1000ms, std::bind(&NonUniformBsplineNode::Run, this));
    }

    void Run()
    {
        showControlPoints();
    }

private:

    void showControlPoints()
    {
        std::string ns = "control_points";
        std::string frame = "map";
        visualization_msgs::msg::Marker marker = visualization_tools::RosVizTools::newSphereList(0.2, ns, 0, visualization_tools::LIME_GREEN, frame);
        std_msgs::msg::ColorRGBA color = visualization_tools::newColorRGBA(0, 255, 0);

        // [(3 , 1), (2.5, 4), (0, 1), (-2.5, 4),(-3, 0), (-2.5, -4), (0, -1), (2.5, -4), (3, -1),]
        geometry_msgs::msg::Point p;
        p.x = 3.0;
        p.y = 1.0;
        marker.points.push_back(p);

        p.x = 2.5;
        p.y = 4.0;
        marker.points.push_back(p);

        p.x = 0;
        p.y = 1.0;
        marker.points.push_back(p);

        p.x = -2.5;
        p.y = 4.0;
        marker.points.push_back(p);

        p.x = -3.0;
        p.y = 0.0;
        marker.points.push_back(p);

        p.x = -2.5;
        p.y = -4.0;
        marker.points.push_back(p);

        p.x = 0;
        p.y = -1.0;
        marker.points.push_back(p);

        p.x = 2.5;
        p.y = -4.0;
        marker.points.push_back(p);

        p.x = 3.0;
        p.y = -1.0;
        marker.points.push_back(p);
        marker.colors.push_back(color);
        
        // Add 
        points_markers_->append(marker);

        // publish
        points_markers_->publish();
    }

    

private:
    rclcpp::TimerBase::SharedPtr timer_ {nullptr};
    std::shared_ptr<visualization_tools::RosVizTools> points_markers_{nullptr};
    std::shared_ptr<NonUniformBspline> non_uniform_bspline_{nullptr};
};

}  // namespace 
}  // namespace bspline


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    // You MUST use the MultiThreadedExecutor to use, well, multiple threads
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<bspline::NonUniformBsplineNode>();


    // Executor add node 
    executor.add_node(node);

    // Spin here
    executor.spin();
    rclcpp::shutdown();
    return 0;
}