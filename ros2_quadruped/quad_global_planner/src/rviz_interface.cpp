

#include "quad_global_planner/rviz_interface.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"


namespace ros2_quadruped {
namespace quad_global_planner {

RVizInterface::RVizInterface() 
    : Node("rviz_interface_node")
{
    // Load rosparams from parameter server
    this->declare_parameter("body_plan", "body_plan");
    this->declare_parameter("body_plan_viz_topic", "visualization/body_plan");
    this->declare_parameter("discrete_body_plan_topic", "discrete_body_plan");
    this->declare_parameter("discrete_body_plan_viz_topic", "visualization/discrete_body_plan");
    this->declare_parameter("map_frame", "map");
    this->declare_parameter("update_rate", 10);

    body_plan_topic_ = this->get_parameter("body_plan").as_string();
    body_plan_viz_topic_ = this->get_parameter("body_plan_viz_topic").as_string();
    discrete_body_plan_topic_ = this->get_parameter("discrete_body_plan_topic").as_string();
    discrete_body_plan_viz_topic_ = this->get_parameter("discrete_body_plan_viz_topic").as_string();
    map_frame_ = this->get_parameter("map_frame").as_string();
    update_rate_ = this->get_parameter("update_rate").as_int();

    // Setup pubs and subs
    body_plan_sub_ = this->create_subscription<quad_msgs::msg::BodyPlan>(
        body_plan_topic_, 10, 
            std::bind(&RVizInterface::bodyPlanCallback, this, std::placeholders::_1));
    discrete_body_plan_sub_ = this->create_subscription<quad_msgs::msg::BodyPlan>(
        discrete_body_plan_topic_, 10, 
            std::bind(&RVizInterface::discreteBodyPlanCallback, this, std::placeholders::_1));

    body_plan_viz_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        body_plan_viz_topic_, rclcpp::SystemDefaultsQoS());

    discrete_body_plan_viz_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        discrete_body_plan_viz_topic_, rclcpp::SystemDefaultsQoS());
}

void RVizInterface::bodyPlanCallback(const quad_msgs::msg::BodyPlan::SharedPtr msg) 
{
  // Initialize Path message to visualize body plan
  nav_msgs::msg::Path body_plan_viz;
  body_plan_viz.header = msg->header;

  // Loop through the BodyPlan message to get the state info and add to private vector
  int length = msg->states.size();
  for (int i=0; i < length; i++) {

    // Load in the pose data directly from the Odometry message
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header = msg->states[i].header;
    pose_stamped.pose = msg->states[i].pose.pose;

    // Add to the path message
    body_plan_viz.poses.push_back(pose_stamped);
  }

  // Publish the full path
  body_plan_viz_pub_->publish(body_plan_viz);
}

void RVizInterface::discreteBodyPlanCallback(const quad_msgs::msg::BodyPlan::SharedPtr msg) 
{
    // Construct Marker message
    visualization_msgs::msg::Marker discrete_body_plan;

    // Initialize the headers and types
    discrete_body_plan.header = msg->header;
    discrete_body_plan.id = 0;
    discrete_body_plan.type = visualization_msgs::msg::Marker::POINTS;

    // Define the shape of the discrete states
    double scale = 0.2;
    discrete_body_plan.scale.x = scale;
    discrete_body_plan.scale.y = scale;
    discrete_body_plan.scale.z = scale;
    discrete_body_plan.color.r = 0.733f;
    discrete_body_plan.color.a = 1.0;

    // Loop through the discrete states
    int length = msg->states.size();
    for (int i=0; i < length; i++) {
        geometry_msgs::msg::Point p;
        p.x = msg->states[i].pose.pose.position.x;
        p.y = msg->states[i].pose.pose.position.y;
        p.z = msg->states[i].pose.pose.position.z;
        discrete_body_plan.points.push_back(p);
    }

    // Publish both interpolated body plan and discrete states
    discrete_body_plan_viz_pub_->publish(discrete_body_plan);
}

}  // namespace quad_global_planner
}  // namespace ros2_quadruped