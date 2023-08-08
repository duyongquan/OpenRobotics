#include "waypoint_generator/waypoint_generator.hpp"
#include "waypoint_generator/bernoulli_curve.hpp"
#include "waypoint_generator/lissajous_curve.hpp"

namespace ros2_controller
{
namespace waypoint_generator
{

VehicleWaypointGenerator::VehicleWaypointGenerator()
    : rclcpp::Node("waypoint_generator"),
      deault_path_type_("rectangle")
{
    path_publisher_ = this->create_publisher<nav_msgs::msg::Path>(
        "plan", rclcpp::SystemDefaultsQoS());

    nav_goal_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 10, 
        std::bind(&VehicleWaypointGenerator::HandleNavGoalMessage, this, std::placeholders::_1));

    path_type_server_ = this->create_service<PathType>("path_generator_type",
        std::bind(&VehicleWaypointGenerator::HandlePathTypeRequest, this, std::placeholders::_1, std::placeholders::_2));

    follow_path_client_ = rclcpp_action::create_client<FollowPath>(
        this->get_node_base_interface(),
        this->get_node_graph_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(),
        "follow_path");

    trajectory_generator_ = std::make_shared<TrajectoryGenerator>();
}

void VehicleWaypointGenerator::HandleNavGoalMessage(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Recevice RVIZ set nav goal command.");

    // Print path type
    DebugString(path_type_);

    // Create a follow path
    auto plan = CreatePath(path_type_);

    if (plan.poses.empty()) {
        RCLCPP_WARN(this->get_logger(), "Create a invaild path");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "path size: %d", plan.poses.size());

    // Publish this path and send a request
    PublishPath(plan);
}

void VehicleWaypointGenerator::HandlePathTypeRequest(
    const std::shared_ptr<PathType::Request> request,
    const std::shared_ptr<PathType::Response> response)
{
    // if (request->type == "circle") {
    //     path_type_ == "circle";
    // } else if (request->type == "rectangle") {
    //     path_type_ = "rectangle";
    // } else if (request->type == "lissajous") {
    //     path_type_ = "lissajous";
    // } else {
    //     path_type_ = deault_path_type_;
    // }

    // // Print path type
    // DebugString(path_type_);

    // // Create a follow path
    // auto plan = CreatePath(path_type_);
    // if (plan.poses.empty()) {
    //     RCLCPP_WARN(this->get_logger(), "Create a invaild path");
    //     return;
    // }

    // // Publish this path and send a request
    // PublishPath(plan);

    path_type_ = request->type;
    RCLCPP_INFO(this->get_logger(), "path_type: %s", path_type_.c_str());

    response->success = true;
}

void VehicleWaypointGenerator::DebugString(const std::string& type)
{
   RCLCPP_INFO(this->get_logger(), "follow path type: %s", type.c_str());
}

void VehicleWaypointGenerator::PublishPath(const nav_msgs::msg::Path& path)
{
    // Publish path RVIZ 
    path_publisher_->publish(path);

    if (!follow_path_client_->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        return;
    }

    auto goal = FollowPath::Goal();
    goal.path = path;
    goal.controller_id = "FollowPath";
    goal.goal_checker_id = "general_goal_checker";

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<FollowPath>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&VehicleWaypointGenerator::HandleFollowPathGoalResponseCallback, 
        this, std::placeholders::_1);

    send_goal_options.feedback_callback =
      std::bind(&VehicleWaypointGenerator::HandleFollowPathFeedbackCallback, 
        this, std::placeholders::_1, std::placeholders::_2);

    send_goal_options.result_callback =
      std::bind(&VehicleWaypointGenerator::HandleFollowPathResultCallback,
        this, std::placeholders::_1);
    
    follow_path_client_->async_send_goal(goal, send_goal_options);
}

nav_msgs::msg::Path VehicleWaypointGenerator::CreatePath(const std::string& type)
{
    nav_msgs::msg::Path plan;
    if (path_type_== "circle") {
        plan =  trajectory_generator_->CreatePath(TrajectoryType::Circle);
    } else if (path_type_ == "epitrochoid") {
        plan =  trajectory_generator_->CreatePath(TrajectoryType::Epitrochoid);
    } else if (path_type_== "infinite") {
        plan =  trajectory_generator_->CreatePath(TrajectoryType::Infinite);
    } else if (path_type_== "square") {
        plan = trajectory_generator_->CreatePath(TrajectoryType::Square);
    } else {
        plan = trajectory_generator_->CreatePath(TrajectoryType::Circle);
    }

    return plan;
}

void VehicleWaypointGenerator::HandleFollowPathGoalResponseCallback(
    GoalHandleFollowPath::SharedPtr goal_handle)
{
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
}

void VehicleWaypointGenerator::HandleFollowPathFeedbackCallback(
    GoalHandleFollowPath::SharedPtr,
    const std::shared_ptr<const FollowPath::Feedback> feedback)
{
    RCLCPP_INFO(this->get_logger(), "Vehicle's speed: %lf distance_to_goal: %lf",
        feedback->speed, feedback->distance_to_goal);
}


void VehicleWaypointGenerator::HandleFollowPathResultCallback(
    const GoalHandleFollowPath::WrappedResult & result)
{
}
}  // namespace waypoint_generator
}  // namespace ros2_controller