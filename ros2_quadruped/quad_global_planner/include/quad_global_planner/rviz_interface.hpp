#ifndef ROS2_QUADRUPED__QUAD_GLOBAL_PLANNER__RVIZ_INTERFACE_HPP_
#define ROS2_QUADRUPED__QUAD_GLOBAL_PLANNER__RVIZ_INTERFACE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "grid_map_core/grid_map_core.hpp"
#include "grid_map_ros/grid_map_ros.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "quad_msgs/msg/body_plan.hpp"

#include <string>
#include <vector>
#include <sstream> //istringstream
#include <iostream> // cout
#include <fstream> // ifstream

namespace ros2_quadruped {
namespace quad_global_planner {

//! A class for interfacing between RViz and planning topics.
/*!
   RVizInterface is a container for all of the logic utilized in the template node.
   The implementation must provide a clean and high level interface to the core algorithm
*/
class RVizInterface : public rclcpp::Node
{
public:
	/**
	 * @brief Constructor for RVizInterface Class
	 * @param[in] nh ROS NodeHandle to publish and subscribe from
	 * @return Constructed object of type RVizInterface
	 */
	RVizInterface();

	/**
	 * @brief Calls ros spinOnce and pubs data at set frequency
	 */
	void spin();

private:
	/**
     * @brief Callback function to handle new body plan data
     * @param[in] Body plan message contining interpolated output of body planner
     */
    void bodyPlanCallback(const quad_msgs::msg::BodyPlan::SharedPtr msg);

    /**
     * @brief Callback function to handle new body plan discrete state data
     * @param[in] Body plan message contining discrete output of body planner
     */
    void discreteBodyPlanCallback(const quad_msgs::msg::BodyPlan::SharedPtr msg);

	/// ROS subscriber for the body plan
	rclcpp::Subscription<quad_msgs::msg::BodyPlan>::SharedPtr body_plan_sub_{nullptr};

	/// ROS subscriber for the body plan
	rclcpp::Subscription<quad_msgs::msg::BodyPlan>::SharedPtr discrete_body_plan_sub_{nullptr};

	/// ROS Publisher for the interpolated body plan vizualization
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr body_plan_viz_pub_{nullptr};

	/// ROS Publisher for the discrete body plan vizualization
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr discrete_body_plan_viz_pub_{nullptr};

	// Load rosparams from parameter server
    std::string body_plan_topic_;
	std::string body_plan_viz_topic_;
	std::string discrete_body_plan_topic_;
	std::string discrete_body_plan_viz_topic_;
	std::string footstep_plan_viz_topic_;

	/// Update rate for sending and receiving data, unused since pubs are called in callbacks
	double update_rate_;

	/// Handle for the map frame
    std::string map_frame_;
};

}  // namespace quad_global_planner
}  // namespace ros2_quadruped

#endif  // ROS2_QUADRUPED__QUAD_GLOBAL_PLANNER__RVIZ_INTERFACE_HPP_
