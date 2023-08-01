#ifndef ROS2_QUADRUPED__QUAD_GLOBAL_PLANNER__GLOBAL_BODY_PLANNER_HPP_
#define ROS2_QUADRUPED__QUAD_GLOBAL_PLANNER__GLOBAL_BODY_PLANNER_HPP_


#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"

// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "quad_msgs/msg/body_plan.hpp"
#include "quad_global_planner/fast_terrain_map.hpp"
#include "quad_global_planner/planning_utils.hpp"
#include "quad_global_planner/planner_class.hpp"
#include "quad_global_planner/rrt_star_connect.hpp"
#include "grid_map_core/grid_map_core.hpp"
#include "grid_map_ros/grid_map_ros.hpp"
#include "grid_map_ros/GridMapRosConverter.hpp"


namespace ros2_quadruped {
namespace quad_global_planner {

using namespace planning_utils;

//! A global body planning class for legged robots
/*!
   GlobalBodyPlanner is a container for all of the logic utilized in the global body planning node.
   This algorithm requires an height map of the terrain as a GridMap message type, and will publish
   the global body plan as a BodyPlan message over a topic. It will also publish the discrete states
   used by the planner (from which the full path is interpolated).
*/
class GlobalBodyPlanner : public rclcpp::Node
{
public:
    /**
     * @brief Constructor for GlobalBodyPlanner Class
     * @return Constructed object of type GlobalBodyPlanner
     */
    GlobalBodyPlanner();

    /**
     * @brief Call the correct planning class and compute statistics
     */
    void callPlanner();

private:
    /**
     * @brief Callback function to handle new terrain map data
     * @param[in] msg the message contining map data
     */
    void terrainMapCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg);

    /**
     * @brief Set the start and goal states of the planner
     */
    void setStartAndGoalStates();

    /**
     * @brief Clear the plan member variables
     */
    void clearPlan();

    /**
     * @brief Update the body plan with the current plan
     * @param[in] t Time of state in trajectory
     * @param[in] body_state Body state
     * @param[in] body_plan_msg Body plan message
     */
    void addBodyStateToMsg(double t, State body_state, quad_msgs::msg::BodyPlan& body_plan_msg);

    /**
     * @brief Publish the current body plan
     */
    void publishPlan();

    /**
     * @brief Wait until a map message has been received and processed
     */
    void waitForMap();

    /**
     * @brief Create a path plan
     */
    void makePlan();

    /// Subscriber for terrain map messages
    rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr terrain_map_sub_{nullptr};

    /// Publisher for body plan messages
    rclcpp::Publisher<quad_msgs::msg::BodyPlan>::SharedPtr body_plan_pub_{nullptr};

    /// Publisher for discrete states in body plan messages
    rclcpp::Publisher<quad_msgs::msg::BodyPlan>::SharedPtr discrete_body_plan_pub_{nullptr};

    /// Publish for plan
    rclcpp::TimerBase::SharedPtr timer_{nullptr};

    /// Topics 
    std::string terrain_map_topic_;
    std::string body_plan_topic_;
    std::string discrete_body_plan_topic_;

    /// Update rate for sending and receiving data;
    double update_rate_;

    /// Number of times to call the planner
    int num_calls_;

    /// Algorithm for planner to run (rrt-connect or rrt-star-connect)
    std::string algorithm_;

    /// Time after which replanning is halted;
    double replan_time_limit_;

    /// Handle for the map frame
    std::string map_frame_;

    /// Struct for terrain map data
    FastTerrainMap terrain_;

    /// Std vector containing the interpolated robot body plan
    std::vector<State> body_plan_;

    /// Std vector containing the interpolated time data
    std::vector<double> t_plan_;

    /// Robot starting state
    State robot_start_;

    /// Robot goal state
    State robot_goal_;
    
    /// Sequence of discrete states in the plan
    std::vector<State> state_sequence_;

    /// Sequence of discrete actions in the plan
    std::vector<Action> action_sequence_;

    /// Vector of cost instances in each planning call (nested STL vectors)
    std::vector<std::vector<double> > cost_vectors_;

    /// Vector of time instances of cost data for each planning call (nested STL vectors)
    std::vector<std::vector<double> > cost_vectors_times_;

    /// Vector of solve times for each planning call
    std::vector<double> solve_time_info_;

    /// Vector of number of vertices for each planning call
    std::vector<int> vertices_generated_info_;

    /// Start create a path plan flag
    bool start_make_plan_{false};
};

}  // namespace quad_global_planner
}  // namespace ros2_quadruped

#endif  // ROS2_QUADRUPED__QUAD_GLOBAL_PLANNER__GLOBAL_BODY_PLANNER_HPP_