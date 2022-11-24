#include "service/tutorials_service_demo1_initial_env.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // You MUST use the MultiThreadedExecutor to use, well, multiple threads
  rclcpp::executors::MultiThreadedExecutor executor;
  auto client = std::make_shared<ros2_tutorials::service::Client>();
  auto server = std::make_shared<ros2_tutorials::service::Server>(); 
                                                        // They will still run on different threads
                                                        // One Node. Two callbacks. Two Threads
  executor.add_node(client);
  executor.add_node(server);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
