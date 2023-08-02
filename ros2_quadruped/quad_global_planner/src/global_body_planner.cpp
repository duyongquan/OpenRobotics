#include "quad_global_planner/global_body_planner.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/quaternion.hpp"

namespace ros2_quadruped {
namespace quad_global_planner {

using namespace planning_utils;
using namespace std::chrono_literals;

GlobalBodyPlanner::GlobalBodyPlanner() 
    : Node("global_body_planner_node")
{
    // Load rosparams from parameter server
    this->declare_parameter("terrain_map", "terrain_map");
    this->declare_parameter("body_plan", "body_plan");
    this->declare_parameter("discrete_body_plan", "discrete_body_plan");
    this->declare_parameter("map_frame", "map");
    this->declare_parameter("update_rate", 1);
    this->declare_parameter("num_calls", 1);
    this->declare_parameter("replan_time_limit", 0.0);
    this->declare_parameter("algorithm", "rrt-connect");

    terrain_map_topic_ = this->get_parameter("terrain_map").as_string();
    body_plan_topic_ = this->get_parameter("body_plan").as_string();
    discrete_body_plan_topic_ = this->get_parameter("discrete_body_plan").as_string();
    terrain_map_topic_ = this->get_parameter("terrain_map").as_string();
    map_frame_ = this->get_parameter("map_frame").as_string();
    num_calls_ = this->get_parameter("num_calls").as_int();
    replan_time_limit_ = this->get_parameter("replan_time_limit").as_double();
    algorithm_ = this->get_parameter("algorithm").as_string();

    // Setup pubs and subs
    body_plan_pub_ = this->create_publisher<quad_msgs::msg::BodyPlan>(
        body_plan_topic_, rclcpp::SystemDefaultsQoS());
    
    discrete_body_plan_pub_ = this->create_publisher<quad_msgs::msg::BodyPlan>(
        discrete_body_plan_topic_, rclcpp::SystemDefaultsQoS());

    terrain_map_sub_ = this->create_subscription<grid_map_msgs::msg::GridMap>(
            terrain_map_topic_, 10, 
                std::bind(&GlobalBodyPlanner::terrainMapCallback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
        1000ms, std::bind(&GlobalBodyPlanner::makePlan, this));
}

void GlobalBodyPlanner::terrainMapCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg) 
{
    if (make_plan_finished_) {
        return;
    }

    // Get the map in its native form
    grid_map::GridMap map;
    grid_map::GridMapRosConverter::fromMessage(*msg, map);

    // Convert to FastTerrainMap structure for faster querying
    terrain_.loadDataFromGridMap(map);

    RCLCPP_INFO(this->get_logger(), "Receive terrain map");

    start_make_plan_ = true;
}

void GlobalBodyPlanner::clearPlan() 
{
    // Clear old solutions
    body_plan_.clear();
    t_plan_.clear();
    solve_time_info_.clear();
    vertices_generated_info_.clear();
    cost_vectors_.clear();
    cost_vectors_times_.clear();
}


void GlobalBodyPlanner::callPlanner() 
{
    // Get the most recent plan parameters and clear the old solutions
    setStartAndGoalStates();
    clearPlan();

    // Initialize statistics variables
    double plan_time;
    int success;
    int vertices_generated;
    double time_to_first_solve;
    double path_duration;
    double total_solve_time = 0;
    double total_vertices_generated = 0;
    double total_path_duration = 0;

    // Set up more objects
    cost_vectors_.reserve(num_calls_);
    cost_vectors_times_.reserve(num_calls_);
    RRTClass rrt_obj;
    RRTConnectClass rrt_connect_obj;
    RRTStarConnectClass rrt_star_connect_obj;

    // Loop through num_calls_ planner calls
    for (int i = 0; i<num_calls_; ++i)
    {
        // Clear out previous solutions and initialize new statistics variables
        state_sequence_.clear();
        action_sequence_.clear();
        std::vector<double> cost_vector;
        std::vector<double> cost_vector_times;

        // Call the appropriate planning method (either RRT-Connect or RRT*-Connect)
        if (algorithm_.compare("rrt-connect") == 0){
            rrt_connect_obj.buildRRTConnect(terrain_, robot_start_, robot_goal_,state_sequence_,action_sequence_, replan_time_limit_);
            rrt_connect_obj.getStatistics(plan_time,success, vertices_generated, time_to_first_solve, cost_vector, cost_vector_times, path_duration);
        } else if (algorithm_.compare("rrt-star-connect") == 0){
            rrt_star_connect_obj.buildRRTStarConnect(terrain_, robot_start_, robot_goal_,state_sequence_,action_sequence_, replan_time_limit_);
            rrt_star_connect_obj.getStatistics(plan_time,success, vertices_generated, time_to_first_solve, cost_vector, cost_vector_times, path_duration);
        } else {
            throw std::runtime_error("Invalid algorithm specified");
        }

        // Handle the statistical data
        cost_vectors_.push_back(cost_vector);
        cost_vectors_times_.push_back(cost_vector_times);

        total_solve_time += plan_time;
        total_vertices_generated += vertices_generated;
        total_path_duration += path_duration;

        std::cout << "Vertices generated: " << vertices_generated << std::endl;
        std::cout << "Solve time: " << plan_time << std::endl;
        std::cout << "Time to first solve: " << time_to_first_solve << std::endl;
        std::cout << "Path length: " << cost_vector.back() << std::endl;

        solve_time_info_.push_back(plan_time);
        vertices_generated_info_.push_back(vertices_generated);
    }

    // Report averaged statistics if num_calls_ > 1
    if (num_calls_ > 1)
    {
        std::cout << "Average vertices generated: " << total_vertices_generated/num_calls_ << std::endl;
        std::cout << "Average solve time: " << total_solve_time/num_calls_ << std::endl;
        std::cout << "Average path duration: " << total_path_duration/num_calls_ << std::endl;
    }

    // Interpolate to get full body plan
    double dt = 0.05;
    std::vector<int> interp_phase;
    getInterpPath(state_sequence_, action_sequence_,dt,body_plan_, t_plan_, interp_phase);
}

void GlobalBodyPlanner::setStartAndGoalStates() 
{
    // Update any relevant planning parameters
    robot_start_ = {0,0,0.4,0,0.1,0,0,0};
    robot_goal_ =  {8,0,0.4,0,0,0,0,0};

    robot_start_[2] += terrain_.getGroundHeight(robot_start_[0], robot_start_[1]);
    robot_goal_[2] += terrain_.getGroundHeight(robot_goal_[0], robot_goal_[1]);
}

void GlobalBodyPlanner::addBodyStateToMsg(double t, State body_state, 
    quad_msgs::msg::BodyPlan& msg) {

    // Represent each state as an Odometry message
    nav_msgs::msg::Odometry state;
    state.header.frame_id = map_frame_;
    state.header.stamp = this->get_clock()->now();
    state.child_frame_id = "dummy";

    // Transform from RPY to quat msg
    tf2::Quaternion quat_tf;
    geometry_msgs::msg::Quaternion quat_msg;
    quat_tf.setRPY(0,body_state[6], atan2(body_state[4],body_state[3]));
    // quat_msg = tf2::toMsg(quat_tf);

    // Load the data into the message
    state.pose.pose.position.x = body_state[0];
    state.pose.pose.position.y = body_state[1];
    state.pose.pose.position.z = body_state[2];
    state.pose.pose.orientation = quat_msg;

    state.twist.twist.linear.x = body_state[3];
    state.twist.twist.linear.y = body_state[4];
    state.twist.twist.linear.z = body_state[5];
    state.twist.twist.angular.x = 0;
    state.twist.twist.angular.y = body_state[6];
    state.twist.twist.angular.z = 0;

    msg.states.push_back(state);
}

void GlobalBodyPlanner::publishPlan() 
{
    // Construct BodyPlan messages
    quad_msgs::msg::BodyPlan body_plan_msg;
    quad_msgs::msg::BodyPlan discrete_body_plan_msg;

    // Initialize the headers and types

    body_plan_msg.header.stamp = this->get_clock()->now();
    body_plan_msg.header.frame_id = map_frame_;
    discrete_body_plan_msg.header = body_plan_msg.header;

    // Loop through the interpolated body plan and add to message
    for (int i=0;i<body_plan_.size(); ++i)
        addBodyStateToMsg(t_plan_[i], body_plan_[i], body_plan_msg);

    // Loop through the discrete states and add to message
    for (int i = 0; i<state_sequence_.size(); i++)
        addBodyStateToMsg(t_plan_[i], state_sequence_[i], discrete_body_plan_msg);
    
    // Publish both interpolated body plan and discrete states
    body_plan_pub_->publish(body_plan_msg);
    discrete_body_plan_pub_->publish(discrete_body_plan_msg);
}

void GlobalBodyPlanner::makePlan()
{
    if (!start_make_plan_) {
        return;
    }

    if (!make_plan_finished_) {
        // Update the plan
        callPlanner();
        make_plan_finished_ = true;
        // start_make_plan_ = false;
    }
 
    publishPlan();
}

}  // namespace quad_global_planner
}  // namespace ros2_quadruped