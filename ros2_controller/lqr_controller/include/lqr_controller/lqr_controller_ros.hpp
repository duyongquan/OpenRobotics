#ifndef ROS2_CONTROLLER__LQR_CONTROLLER__LQR_QUADRATIC_ROS_HPP_
#define ROS2_CONTROLLER__LQR_CONTROLLER__LQR_QUADRATIC_ROS_HPP_

#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <mutex>

#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

namespace ros2_controller {
namespace lqr_controller {

/**
 * @class LQRController::RegulatedPurePursuitController
 * @brief LQRController controller plugin
 */
class LQRController : public nav2_core::Controller
{
public:
    /**
     * @brief Constructor for LQRController::RegulatedPurePursuitController
     */
    LQRController() = default;

    /**
     * @brief Destrructor for LQRController::RegulatedPurePursuitController
     */
    ~LQRController() override = default;

    /**
     * @brief Configure controller state machine
     * @param parent WeakPtr to node
     * @param name Name of plugin
     * @param tf TF buffer
     * @param costmap_ros Costmap2DROS object of environment
     */
    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
        std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

    /**
     * @brief Cleanup controller state machine
     */
    void cleanup() override;

    /**
     * @brief Activate controller state machine
     */
    void activate() override;

    /**
     * @brief Deactivate controller state machine
     */
    void deactivate() override;

    /**
     * @brief Compute the best command given the current pose and velocity, with possible debug information
     *
     * Same as above computeVelocityCommands, but with debug results.
     * If the results pointer is not null, additional information about the twists
     * evaluated will be in results after the call.
     *
     * @param pose      Current robot pose
     * @param velocity  Current robot velocity
     * @param goal_checker   Ptr to the goal checker for this task in case useful in computing commands
     * @return          Best command
     */
    geometry_msgs::msg::TwistStamped computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped & pose,
        const geometry_msgs::msg::Twist & velocity,
        nav2_core::GoalChecker * /*goal_checker*/) override;

    /**
     * @brief nav2_core setPlan - Sets the global plan
     * @param path The global plan
     */
    void setPlan(const nav_msgs::msg::Path & path) override;

    /**
     * @brief Limits the maximum linear speed of the robot.
     * @param speed_limit expressed in absolute value (in m/s)
     * or in percentage from maximum robot speed.
     * @param percentage Setting speed limit in percentage if true
     * or in absolute values in false case.
     */
    void setSpeedLimit(const double & speed_limit, const bool & percentage) override;
};


}  // namespace lqr_controller
}  // namespace ros2_controller


#endif  // ROS2_CONTROLLER__LQR_CONTROLLER__LQR_QUADRATIC_ROS_HPP_