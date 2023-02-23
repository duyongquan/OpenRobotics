#ifndef ROS2_TUTORIALS__TUTORIALS_TOPIC_DEMO6_MEMBER_FUNCTION_WITH_UNIQUE_NETWORK_FLOW_ENDPOINTS_HPP_
#define ROS2_TUTORIALS__TUTORIALS_TOPIC_DEMO6_MEMBER_FUNCTION_WITH_UNIQUE_NETWORK_FLOW_ENDPOINTS_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


namespace ros2_tutorials
{
namespace topic
{

class MinimalPublisherWithUniqueNetworkFlowEndpoints : public rclcpp::Node
{
public:
  MinimalPublisherWithUniqueNetworkFlowEndpoints();

private:
  void timer_1_callback();
  void timer_2_callback();

  /// Print network flow endpoints in JSON-like format
  void print_network_flow_endpoints(
    const std::vector<rclcpp::NetworkFlowEndpoint> & network_flow_endpoints) const;

  rclcpp::TimerBase::SharedPtr timer_1_;
  rclcpp::TimerBase::SharedPtr timer_2_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_1_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_2_;
  size_t count_1_;
  size_t count_2_;
};

class MinimalSubscriberWithUniqueNetworkFlowEndpoints : public rclcpp::Node
{
public:
  MinimalSubscriberWithUniqueNetworkFlowEndpoints();

private:
  void topic_1_callback(const std_msgs::msg::String::SharedPtr msg) const;
  void topic_2_callback(const std_msgs::msg::String::SharedPtr msg) const;

  /// Print network flow endpoints in JSON-like format
  void print_network_flow_endpoints(
    const std::vector<rclcpp::NetworkFlowEndpoint> & network_flow_endpoints) const;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_1_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_2_;
};

}  // namespace topic
}  // namespace ros2_tutorials

#endif  // ROS2_TUTORIALS__TUTORIALS_TOPIC_DEMO6_MEMBER_FUNCTION_WITH_UNIQUE_NETWORK_FLOW_ENDPOINTS_HPP_