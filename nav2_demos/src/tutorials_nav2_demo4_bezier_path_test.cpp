#include <chrono>
#include <memory>
#include <string>
#include <cmath>
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/path.hpp"

#include "nav2_demos/tutorials_nav2_utils_poses_publisher.hpp"

using namespace std::chrono_literals;

namespace ros2_tutorials
{
namespace nav2
{
namespace
{

class BezierPathNode : public rclcpp::Node
{
public:
    BezierPathNode() : Node("bezier_path")
    {
        pose_publisher_ = std::make_shared<PosesPublisher>(this);
        timer_ = create_wall_timer(
            1000ms, std::bind(&BezierPathNode::HandleTimerCallback, this));
    }

    ~BezierPathNode() {}

private:

    void HandleTimerCallback()
    {
        int interpolation_order = 100;
        BezierCurve(interpolation_order);
    }

    void BezierCurve(int interpolation_order)
    {
        std::vector<Eigen::Vector2d> points {
            Eigen::Vector2d (0,0),
            Eigen::Vector2d(1,1),
            Eigen::Vector2d(2,1),
            Eigen::Vector2d(3,0),
            Eigen::Vector2d(4,2)
        };

        auto reference_points = ToROSReferencePoints(points);
        ShowBezierCurveReferencePoints(reference_points);

        std::vector<Eigen::Vector2d> eigen_points;
        for(int t = 0; t < interpolation_order; t++){
            Eigen::Vector2d pose = BezierCommon(points, (double)t / interpolation_order);
            eigen_points.push_back(pose);
        }
        ShowBezierCurve(ToROS(eigen_points));
    }


    void ShowBezierCurve(const sensor_msgs::msg::PointCloud& points)
    {
        pose_publisher_->PublishPoses(points);
    }

    void ShowBezierCurveReferencePoints(const geometry_msgs::msg::PoseArray& poses)
    {
        pose_publisher_->PublishReferencePoses(poses);
    }


    sensor_msgs::msg::PointCloud ToROS(const std::vector<Eigen::Vector2d>& eigen_points)
    {
        sensor_msgs::msg::PointCloud points;
        points.header.frame_id = "map";
        points.header.stamp = this->get_clock()->now();

        for (long unsigned i = 0; i < eigen_points.size(); i++) {
            geometry_msgs::msg::Point32 point;
            point.x = eigen_points[i].x();
            point.y = eigen_points[i].y();
            points.points.push_back(point);
        }
        return points;
    }

    geometry_msgs::msg::PoseArray ToROSReferencePoints(const std::vector<Eigen::Vector2d>& eigen_points)
    {
        geometry_msgs::msg::PoseArray poses;
        poses.header.frame_id = "map";
        poses.header.stamp = this->get_clock()->now();

        for (auto const point : eigen_points) {
            geometry_msgs::msg::Pose pose;
            pose.position.x = point.x();
            pose.position.y = point.y();
            poses.poses.push_back(pose);
        }
        return poses;
    }

    /**
     * 阶乘实现
     */
    double Factorial(int n)
    {
        if(n <= 1) return 1;
            return Factorial(n - 1) * n;
    }
   
    /**
     * 贝塞尔公式
     */
    Eigen::Vector2d BezierCommon(const std::vector<Eigen::Vector2d>& points,  double t)
    {
        if(points.size() == 1) {
            return points[0];
        }

        Eigen::Vector2d p_t(0., 0.);
        int n = points.size() - 1;

        for(long unsigned i = 0; i < points.size(); i++) {
            double C_n_i = Factorial(n) / (Factorial(i) * Factorial(n-i));
            p_t +=  C_n_i*pow((1 - t),(n - i)) * pow(t,i) * points[i];
        }
        return p_t;
    }

    rclcpp::TimerBase::SharedPtr timer_ {nullptr};
    std::shared_ptr<PosesPublisher> pose_publisher_ {nullptr};
};  
}


}  // namespace nav2
}  // namespace ros2_tutorials



int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ros2_tutorials::nav2::BezierPathNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
