#include "topic/tutorials_topic_demo6_member_function_with_unique_network_flow_endpoints.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace ros2_tutorials
{
namespace topic
{

MinimalPublisherWithUniqueNetworkFlowEndpoints::MinimalPublisherWithUniqueNetworkFlowEndpoints()
: Node("minimal_publisher_with_unique_network_flow_endpoints"), count_1_(0), count_2_(0)
{
    // Create publisher with unique network flow endpoints
    // Enable unique network flow endpoints via options
    auto options_1 = rclcpp::PublisherOptions();
    options_1.require_unique_network_flow_endpoints =
        RMW_UNIQUE_NETWORK_FLOW_ENDPOINTS_OPTIONALLY_REQUIRED;
    publisher_1_ = this->create_publisher<std_msgs::msg::String>("topic_1", 10, options_1);
    timer_1_ = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisherWithUniqueNetworkFlowEndpoints::timer_1_callback, this));

    // Create publisher without unique network flow endpoints
    // Unique network flow endpoints are disabled in default options
    auto options_2 = rclcpp::PublisherOptions();
    publisher_2_ = this->create_publisher<std_msgs::msg::String>("topic_2", 10);
    timer_2_ = this->create_wall_timer(
        1000ms, std::bind(&MinimalPublisherWithUniqueNetworkFlowEndpoints::timer_2_callback, this));

    // Get network flow endpoints
    auto network_flow_endpoints_1 = publisher_1_->get_network_flow_endpoints();
    auto network_flow_endpoints_2 = publisher_2_->get_network_flow_endpoints();

    // Print network flow endpoints
    print_network_flow_endpoints(network_flow_endpoints_1);
    print_network_flow_endpoints(network_flow_endpoints_2);
}


void MinimalPublisherWithUniqueNetworkFlowEndpoints::timer_1_callback()
{
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_1_++);

    RCLCPP_INFO(
        this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_1_->publish(message);
}

void MinimalPublisherWithUniqueNetworkFlowEndpoints::timer_2_callback()
{
    auto message = std_msgs::msg::String();
    message.data = "Hej, världen! " + std::to_string(count_2_++);

    RCLCPP_INFO(
        this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_2_->publish(message);
}

/// Print network flow endpoints in JSON-like format
void MinimalPublisherWithUniqueNetworkFlowEndpoints::print_network_flow_endpoints(
    const std::vector<rclcpp::NetworkFlowEndpoint> & network_flow_endpoints) const
{
    std::ostringstream stream;
    stream << "{\"networkFlowEndpoints\": [";
    bool comma_skip = true;
    for (auto network_flow_endpoint : network_flow_endpoints) {
        if (comma_skip) {
        comma_skip = false;
        } else {
        stream << ",";
        }
        stream << network_flow_endpoint;
    }
    stream << "]}";
    RCLCPP_INFO(
        this->get_logger(), "%s",
        stream.str().c_str());
}
  

MinimalSubscriberWithUniqueNetworkFlowEndpoints::MinimalSubscriberWithUniqueNetworkFlowEndpoints()
: Node("minimal_subscriber_with_unique_network_flow_endpoints")
{
    try 
    {
        // Create subscription with unique network flow endpoints
        // Enable unique network flow endpoints via options
        // Since option is strict, expect exception
        auto options_1 = rclcpp::SubscriptionOptions();
        options_1.require_unique_network_flow_endpoints =
        RMW_UNIQUE_NETWORK_FLOW_ENDPOINTS_STRICTLY_REQUIRED;

        subscription_1_ = this->create_subscription<std_msgs::msg::String>(
        "topic_1", 10, std::bind(
            &MinimalSubscriberWithUniqueNetworkFlowEndpoints::topic_1_callback, this,
            _1), options_1);

        // Create subscription without unique network flow endpoints
        // Unique network flow endpoints are disabled by default
        auto options_2 = rclcpp::SubscriptionOptions();
        subscription_2_ = this->create_subscription<std_msgs::msg::String>(
        "topic_2", 10, std::bind(
            &MinimalSubscriberWithUniqueNetworkFlowEndpoints::topic_2_callback, this,
            _1), options_2);

        // Get network flow endpoints
        auto network_flow_endpoints_1 = subscription_1_->get_network_flow_endpoints();
        auto network_flow_endpoints_2 = subscription_2_->get_network_flow_endpoints();

        // Check if network flow endpoints are unique
        for (auto network_flow_endpoint_1 : network_flow_endpoints_1) {
        for (auto network_flow_endpoint_2 : network_flow_endpoints_2) {
            if (network_flow_endpoint_1 == network_flow_endpoint_2) {
            RCLCPP_ERROR(
                this->get_logger(), "Network flow endpoints across subscriptions are not unique");
            break;
            }
        }
        }

        // Print network flow endpoints
        print_network_flow_endpoints(network_flow_endpoints_1);
        print_network_flow_endpoints(network_flow_endpoints_2);
    } catch (const rclcpp::exceptions::RCLError & e) {
        RCLCPP_ERROR(
        this->get_logger(),
        "Error: %s",
        e.what());
    }
}


void MinimalSubscriberWithUniqueNetworkFlowEndpoints::topic_1_callback(
    const std_msgs::msg::String::SharedPtr msg) const
{
    RCLCPP_INFO(this->get_logger(), "Topic 1 news: '%s'", msg->data.c_str());
}

void MinimalSubscriberWithUniqueNetworkFlowEndpoints::topic_2_callback(
    const std_msgs::msg::String::SharedPtr msg) const
{
    RCLCPP_INFO(this->get_logger(), "Topic 2 news: '%s'", msg->data.c_str());
}

/// Print network flow endpoints in JSON-like format
void MinimalSubscriberWithUniqueNetworkFlowEndpoints::print_network_flow_endpoints(
    const std::vector<rclcpp::NetworkFlowEndpoint> & network_flow_endpoints) const
{
    std::ostringstream stream;
    stream << "{\"networkFlowEndpoints\": [";
    bool comma_skip = true;
    for (auto network_flow_endpoint : network_flow_endpoints) {
        if (comma_skip) {
        comma_skip = false;
        } else {
        stream << ",";
        }
        stream << network_flow_endpoint;
    }
    stream << "]}";
    RCLCPP_INFO(
        this->get_logger(), "%s",
        stream.str().c_str());
}

}  // namespace topic
}  // namespace ros2_tutorials