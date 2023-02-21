#include "topic/tutorials_topic_demo2_member_function.hpp"

// int main(int argc, char ** argv)
// {
//   rclcpp::init(argc, argv);
//   auto publisher_node = std::make_shared<ros2_tutorials::topic::MinimalPublisher>();
//   rclcpp::spin(publisher_node);
//   rclcpp::shutdown();
//   return 0;
// }

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // You MUST use the MultiThreadedExecutor to use, well, multiple threads
  rclcpp::executors::MultiThreadedExecutor executor;
  auto publisher_node = std::make_shared<ros2_tutorials::topic::MinimalPublisher>();
  auto subscriber_node = std::make_shared<ros2_tutorials::topic::MinimalSubscriber>();  // This contains BOTH subscriber callbacks.
                                                        // They will still run on different threads
                                                        // One Node. Two callbacks. Two Threads
  executor.add_node(publisher_node);
  executor.add_node(subscriber_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
