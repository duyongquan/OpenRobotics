#include "service/tutorials_service_demo1_initial_env.hpp"

using namespace std::chrono_literals;


namespace ros2_tutorials
{
namespace service
{

Client::Client()
: Node("minimal_client")
{
    client_ = this->create_client<AddTwoInts>("add_two_ints");
    timer_ = this->create_wall_timer(
        2s, std::bind(&Client::HandleTimerCallback, this));
}

void Client::HandleTimerCallback()
{
    if (!connect_) {
        connect_ = ConnectServer(std::chrono::seconds(5));
        return;
    }

    SendMessage();
}

Client::~Client()
{
}

bool Client::ConnectServer(const std::chrono::seconds & timeout)
{
    while (!client_->wait_for_service(timeout)) {
        if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");
    }
    return true;
}

void Client::SendMessage()
{
    auto request = std::make_shared<AddTwoInts::Request>();
    request->a = 41;
    request->b = 1;
    auto result_future = client_->async_send_request(request);
}

Server::Server()
: Node("minimal_server")
{
    server_ = this->create_service<AddTwoInts>("add_two_ints",
        std::bind(&Server::HandleClientRequest, this, std::placeholders::_1, std::placeholders::_2));
}

Server::~Server()
{
}

void Server::HandleClientRequest(
    const std::shared_ptr<AddTwoInts::Request> request,
    const std::shared_ptr<AddTwoInts::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Incoming request a: %d,  b: %d", request->a, request->b);
    response->sum = request->a + request->b;
}

}  // namespace service
}  // namespace ros2_tutorials

