#ifndef ROS2_TUTORIALS__TUTORIALS_SERVICE_DEMO1_INITIAL_ENV_HPP_
#define ROS2_TUTORIALS__TUTORIALS_SERVICE_DEMO1_INITIAL_ENV_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tutorials_msgs/srv/add_two_ints.hpp"

namespace ros2_tutorials
{
namespace service
{

using AddTwoInts = tutorials_msgs::srv::AddTwoInts;

class Client : public rclcpp::Node
{
public:
    Client();
    ~Client();

    bool ConnectServer(const std::chrono::seconds & timeout);
    void SendMessage();

private:
    void HandleTimerCallback();

    rclcpp::Client<AddTwoInts>::SharedPtr client_{nullptr};
    rclcpp::TimerBase::SharedPtr timer_ {nullptr};
    bool connect_{false};

};

class Server : public rclcpp::Node
{
public:
    Server();
    ~Server();

private:
    void HandleClientRequest(
        const std::shared_ptr<AddTwoInts::Request> request,
        const std::shared_ptr<AddTwoInts::Response> response);

    rclcpp::Service<AddTwoInts>::SharedPtr server_{nullptr};
};

}  // namespace service
}  // namespace ros2_tutorials

#endif  // ROS2_TUTORIALS__TUTORIALS_SERVICE_DEMO1_INITIAL_ENV_HPP_