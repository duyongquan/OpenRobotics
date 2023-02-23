#include "action/tutorials_action_demo1_member_functions.hpp"

// int main(int argc, char * argv[])
// {
//   rclcpp::init(argc, argv);

//   rclcpp::executors::MultiThreadedExecutor executor;
//   auto action_server = std::make_shared<ros2_tutorials::action::MinimalActionServer>();
//   auto action_client = std::make_shared<ros2_tutorials::action::MinimalActionClient>();

//   while (!action_client->is_goal_done()) {
//     executor.add_node(action_server);
//     executor.add_node(action_client);
//   }

//   executor.spin();
//   rclcpp::shutdown();
//   return 0;
// }

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto action_server = std::make_shared<ros2_tutorials::action::MinimalActionClient>();

  rclcpp::spin(action_server);

  rclcpp::shutdown();
  return 0;
}

