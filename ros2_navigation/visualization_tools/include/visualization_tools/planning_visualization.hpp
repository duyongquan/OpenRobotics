#ifndef VISUALIZATION_TOOLS__PLANNING_VISUALIZATION_HPP_
#define VISUALIZATION_TOOLS__PLANNING_VISUALIZATION_HPP_

#include "visualization_tools/color.hpp"
#include "bspline/non_uniform_bspline.hpp"
#include "trajectory/polynomial_trajectory.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <Eigen/Eigen>
#include <string>
#include <vector>
#include <list>
#include <iostream>
#include <algorithm>

namespace visualization_tools 
{

class PlanningVisualization 
{
public:
    PlanningVisualization(rclcpp::Node *node);

    // draw basic shapes
    void displaySphereList(const std::vector<Eigen::Vector3d>& list, double resolution,
                           const Eigen::Vector4d& color, int id, int pub_id = 0);

    void displayCubeList(const std::vector<Eigen::Vector3d>& list, double resolution,
                        const Eigen::Vector4d& color, int id, int pub_id = 0);

    void displayLineList(const std::vector<Eigen::Vector3d>& list1, 
                         const std::vector<Eigen::Vector3d>& list2,
                         double line_width, 
                         const Eigen::Vector4d& color, int id, int pub_id = 0);

    // draw a piece-wise straight line path
    void drawGeometricPath(const std::vector<Eigen::Vector3d>& path, 
                           double resolution,
                           const Eigen::Vector4d& color, int id = 0);

    // draw a polynomial trajectory
    void drawPolynomialTraj(trajectory::PolynomialTraj poly_traj, double resolution, const Eigen::Vector4d& color, int id = 0);

    // draw a bspline trajectory
    void drawBspline(bspline::NonUniformBspline& bspline, double size, const Eigen::Vector4d& color,
                    bool show_ctrl_pts = false, double size2 = 0.1,
                    const Eigen::Vector4d& color2 = Eigen::Vector4d(1, 1, 0, 1), int id1 = 0,
                    int id2 = 0);

    // draw a set of bspline trajectories generated in different phases
    void drawBsplinesPhase1(std::vector<bspline::NonUniformBspline>& bsplines, double size);
    void drawBsplinesPhase2(std::vector<bspline::NonUniformBspline>& bsplines, double size);

    // draw topological graph and paths
    // void drawTopoGraph(std::list<GraphNode::Ptr>& graph, double point_size, double line_width,
    //                    const Eigen::Vector4d& color1, const Eigen::Vector4d& color2,
    //                    const Eigen::Vector4d& color3, int id = 0);

    void drawTopoPathsPhase1(std::vector<vector<Eigen::Vector3d>>& paths, double line_width);
    void drawTopoPathsPhase2(std::vector<vector<Eigen::Vector3d>>& paths, double line_width);

    void drawGoal(Eigen::Vector3d goal, double resolution, const Eigen::Vector4d& color, int id = 0);
    // void drawPrediction(ObjPrediction pred, double resolution, const Eigen::Vector4d& color, int id = 0);

    Eigen::Vector4d getColor(double h, double alpha = 1.0);

    // typedef std::shared_ptr<PlanningVisualization> Ptr;

    // SECTION developing
    void drawYawTraj(bspline::NonUniformBspline& pos, bspline::NonUniformBspline& yaw, const double& dt);
    void drawYawPath(bspline::NonUniformBspline& pos, const std::vector<double>& yaw, const double& dt);

private:
    enum TrajectoryPlanningId 
    {
        GOAL = 1,
        PATH = 200,
        BSPLINE = 300,
        BSPLINE_CTRL_PT = 400,
        POLY_TRAJ = 500
    };

    enum TopologicalPathPlanningId 
    {
        GRAPH_NODE = 1,
        GRAPH_EDGE = 100,
        RAW_PATH = 200,
        FILTERED_PATH = 300,
        SELECT_PATH = 400
    };

    rclcpp::Node *node_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr traj_pub_;      // 0
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr topo_pub_;      // 1
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr predict_pub_;   // 2
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr visib_pub_;     // 3, visibility constraints
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr frontier_pub_;  // 4, frontier searching
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr yaw_pub_;       // 5, yaw trajectory
    std::vector<rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr> pubs_; 

    int last_topo_path1_num_;
    int last_topo_path2_num_;
    int last_bspline_phase1_num_;
    int last_bspline_phase2_num_;
    int last_frontier_num_;
};

}   //  namespace visualization_tools 
#endif   // VISUALIZATION_TOOLS__PLANNING_VISUALIZATION_HPP_