#ifndef ROS2_QUADRUPED__QUAD_SIMULATOR_QUAD_UTILS_RVIZ_INTERFACE_NODE_HPP_
#define ROS2_QUADRUPED__QUAD_SIMULATOR_QUAD_UTILS_RVIZ_INTERFACE_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "quad_utils/rviz_interface.hpp"

#include <memory>

namespace ros2_quadruped {
namespace quad_utils {

class RVIZPublisher : public rclcpp::Node
{
public:
    RVIZPublisher();
    ~RVIZPublisher();

private:
    std::shared_ptr<RVizInterface> rviz_interface_{nullptr};
};

}  // namespace quad_utils
}  // namespace ros2_quadruped

#endif  // ROS2_QUADRUPED__QUAD_SIMULATOR_QUAD_UTILS_RVIZ_INTERFACE_NODE_HPP_