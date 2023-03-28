/*
copyright
*/
#ifndef EXAMPLE_ACTIONI_RCLCPP_ROBOT_H_
#define EXAMPLE_ACTIONI_RCLCPP_ROBOT_H_
#include "rclcpp/rclcpp.hpp"
#include "robot_control_interfaces/action/move_robot.hpp"

class Robot {
 public:
  //MoveRobot is action interface
  using MoveRobot = robot_control_interfaces::action::MoveRobot;
  Robot() = default;
  ~Robot() = default;

  bool set_goal(float distance); // move distance

  float move_step(); // move a small step, 500ms call the func
  float get_current_pose() const;

  int get_status() const;
  bool close_goal(); // Is close target
  void stop_move(); // stop move

 private:
  float current_pose_ {}; // cunrent position
  float target_pose_ {}; // target distance 
  float move_distance_ {}; // moved distance
  
  int status_ = MoveRobot::Feedback::STATUS_STOP;
};

#endif  // EXAMPLE_ACTIONI_RCLCPP_ROBOT_H_
