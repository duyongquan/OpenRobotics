#include <chrono>
#include <memory>
#include <string>
#include <cmath>

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

struct Point
{
    double x;
    double y;
    double yaw;
};

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
        auto points = CreatePoints();
        pose_publisher_->PublishPoses(points);
    }

    sensor_msgs::msg::PointCloud CreatePoints()
    {
        sensor_msgs::msg::PointCloud points;
        points.header.frame_id = "map";
        points.header.stamp = this->get_clock()->now();

        for (int i = 0; i < 10; i++) {
            geometry_msgs::msg::Point32 point;
            point.x = i;
            point.y = pow(i, 2);
            points.points.push_back(point);
        }
        return points;
    }

    void Show()
    {
        // Plot an example bezier curve.
        // double start_x = 10.0;      // [m]
        // double start_y = 1.0;       // [m]
        // double start_yaw = M_PI;    // [rad]

        // double end_x = -0.0;        // [m]
        // double end_y = -3.0;        // [m]
        // double end_yaw = -M_PI/4;   // [rad]
        double offset = 3.0;
        Point start = {10.0, 1.0, M_PI};
        Point end = {-0.0, -3.0, -M_PI/4};
    }

    std::vector<Point> Calc4PointsBezierPath(
        const Point& start, const Point& end, const double offset)
    {
        // control_points 
        auto control_points = CreateControlPoints(start, end, offset);
        auto path = CalcBezierPath(control_points, 100);
    }


    std::vector<Point> CreateControlPoints(const Point& start, const Point& end, const double offset)
    {
        std::vector<Point> points;
        double dist = std::hypot(start.x - end.x, start.y - end.y) / offset;
        points.push_back(start);
        points.push_back(Point{
            start.x + dist * std::cos(start.yaw),
            start.y + dist * std::sin(start.yaw),
        });

        points.push_back(Point{
            end.x - dist * std::cos(end.yaw),
            end.y + dist * std::sin(end.yaw),
        });
        points.push_back(end);
        return points;
    }

    std::vector<Point> CalcBezierPath(const std::vector<Point>& control_points, double n_points)
    {
        std::vector<Point> path;

        // for t in np.linspace(0, 1, n_points):
        // traj.append(bezier(t, control_points))

        double step = 1.0 / n_points;
        
        for (int i = 0; i <= n_points; i++) {
            Bezier(i / n_points, control_points);
            // path.push_back(0.0);
        }

        return path;
    }

    double Bezier(double t, const std::vector<Point>& control_points)
    {
        // n = len(control_points) - 1
        // return np.sum([bernstein_poly(n, i, t) * control_points[i] for i in range(n + 1)], axis=0)
        double sum = 0.0f;
        int n = control_points.size() - 1;
        for (int i = 0; i < n; i++) {
            sum += BernsteinPoly(n, i, t) * control_points[i].x;
        }
        return sum;
    }

    /**
     * Bernstein polynom
     * 
     * @param n: (int) polynom degree
     * @param i: (int)
     * @param t: (double)
     * @param n: (int) polynom degree
     * 
     * @return: (double)
     */
    double BernsteinPoly(int n, int i, double t)
    {
        // return scipy.special.comb(n, i) * t ** i * (1 - t) ** (n - i)
        return BinomialCoefficients(n, i) * std::pow(t, i) * std::pow(1 - t, n - i);
    }

    // c(n,k) = c(n-1 , k-1) + c(n-1, k)
    // c(n, 0) = c(n, n) = 1
    int BinomialCoefficients(int n, int k) {
        if (k == 0 || k == n)
            return 1;
        return BinomialCoefficients(n - 1, k - 1) + BinomialCoefficients(n - 1, k);
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
