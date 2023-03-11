#include "quad_utils/rviz_interface_node.hpp"

using namespace std::chrono_literals;

namespace ros2_quadruped {
namespace quad_utils {

RVIZPublisher::RVIZPublisher() : rclcpp::Node("rviz_interface_publisher")
{
    rviz_interface_ = std::make_shared<RVizInterface>(this);
}

RVIZPublisher::~RVIZPublisher()
{
}

}  // namespace quad_utils
}  // namespace ros2_quadruped