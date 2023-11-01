
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "trajectory/polynomial_trajectory.hpp"

using namespace std::chrono_literals;

namespace trajectory
{
namespace
{

class TrajectoryGeneratorNode : public rclcpp::Node
{
public:
    TrajectoryGeneratorNode() : Node("tractory_generator_node")
    {
        traj_pub = this->create_publisher<visualization_msgs::msg::Marker>("/traj_generator/traj_vis", 10);

        // timer
        timer_ = create_wall_timer(1000ms, std::bind(&TrajectoryGeneratorNode::Run, this));
    }

    void Run()
    {
        Eigen::MatrixXd pos(9, 3);
        pos.row(0) = Eigen::Vector3d(0, 0, 0);
        pos.row(1) = Eigen::Vector3d(-0.5, 0.5, 1);
        pos.row(2) = Eigen::Vector3d(0, 0, 1);
        pos.row(3) = Eigen::Vector3d(0.5, -0.5, 1);
        pos.row(4) = Eigen::Vector3d(1, 0, 1);
        pos.row(5) = Eigen::Vector3d(0.5, 0.5, 1);
        pos.row(6) = Eigen::Vector3d(0, 0, 1);
        pos.row(7) = Eigen::Vector3d(-0.5, -0.5, 1);
        pos.row(8) = Eigen::Vector3d(-1, 0, 1);

        Eigen::VectorXd time(8);
        time(0) = 2.0;
        time(1) = 1.5;
        time(2) = 1.5;
        time(3) = 1.5;
        time(4) = 1.5;
        time(5) = 1.5;
        time(6) = 1.5;
        time(7) = 2.0;

        // Eigen::MatrixXd poly = generateTraj(pos, time);
        // std::cout << "poly:\n" << poly << std::endl;
        // std::cout << "pos:\n" << pos << std::endl;
        // std::cout << "pos 0 1 2: " << pos(0) << ", " << pos(1) << ", " << pos(2) << std::endl;

        // /* ---------- use polynomials ---------- */
        // PolynomialTraj poly_traj;
        // for (int i = 0; i < poly.rows(); ++i) 
        // {
        //     std::vector<double> cx(6), cy(6), cz(6);
        //     for (int j = 0; j < 6; ++j) {
        //         cx[j] = poly(i, j);
        //         cy[j] = poly(i, j + 6);
        //         cz[j] = poly(i, j + 12);
        //     }
        //     reverse(cx.begin(), cx.end());
        //     reverse(cy.begin(), cy.end());
        //     reverse(cz.begin(), cz.end());
        //     double ts = time(i);
        //     poly_traj.addSegment(cx, cy, cz, ts);
        // }
        // std::vector<Eigen::Vector3d> traj_vis = poly_traj.getTraj();
        // displayPathWithColor(traj_vis, 0.05, Eigen::Vector4d(1, 0, 0, 1), 1);        
    }

private:

    void displayPathWithColor(std::vector<Eigen::Vector3d> path, double resolution, Eigen::Vector4d color, int id) 
    {
        visualization_msgs::msg::Marker mk;
        mk.header.frame_id = "world";
        mk.header.stamp = rclcpp::Clock().now();
        mk.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        mk.action = visualization_msgs::msg::Marker::DELETE;
        mk.id = id;

        mk.action = visualization_msgs::msg::Marker::ADD;
        mk.pose.orientation.x = 0.0;
        mk.pose.orientation.y = 0.0;
        mk.pose.orientation.z = 0.0;
        mk.pose.orientation.w = 1.0;

        mk.color.r = color(0);
        mk.color.g = color(1);
        mk.color.b = color(2);
        mk.color.a = color(3);

        mk.scale.x = resolution;
        mk.scale.y = resolution;
        mk.scale.z = resolution;

        geometry_msgs::msg::Point pt;
        for (int i = 0; i < int(path.size()); i++) {
            pt.x = path[i](0);
            pt.y = path[i](1);
            pt.z = path[i](2);
            mk.points.push_back(pt);
        }
        traj_pub->publish(mk);
    }

private:
    rclcpp::TimerBase::SharedPtr timer_ {nullptr};
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr state_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr traj_pub;
};

}  // namespace 
}  // namespace trajectory


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    // You MUST use the MultiThreadedExecutor to use, well, multiple threads
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<trajectory::TrajectoryGeneratorNode>();


    // Executor add node 
    executor.add_node(node);

    // Spin here
    executor.spin();
    rclcpp::shutdown();
    return 0;
}