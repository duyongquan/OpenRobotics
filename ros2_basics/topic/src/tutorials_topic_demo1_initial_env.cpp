#include "topic/tutorials_topic_demo1_initial_env.hpp"

namespace ros2_tutorials
{
namespace topic
{

EmptyNode::EmptyNode()
: Node("empty")
{
    RCLCPP_INFO(get_logger(), "Creating empty node.");
}

EmptyNode::~EmptyNode()
{

}

}  // namespace topic
}  // namespace ros2_tutorials