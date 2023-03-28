# include "rclcpp/rclcpp.hpp"
# include "rclcpp_action/rclcpp_action.hpp"
# include "robot_control_interfaces/action/move_robot.hpp"

class ActionControl01 : public rclcpp::Node {
 public:
  // action 接口
  using MoveRobot = robot_control_interfaces::action::MoveRobot;
  
  // 客户端与目标相关的内容：包括目标
  using GoalHandleMoveRobot = rclcpp_action::ClientGoalHandle<MoveRobot>;

  explicit ActionControl01(std::string name):Node(name){
    RCLCPP_INFO(this->get_logger(), "node has been started: %s.", name.c_str());

    this->client_ptr_ = rclcpp_action::create_client<MoveRobot>(this, "move_robot");

    this->timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
                                           std::bind(&ActionControl01::send_goal, this));
  }

  void send_goal() {
    using namespace std::placeholders;
    
    this->timer_->cancel();// 只执行一次send_goal

    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))){
       RCLCPP_ERROR(this->get_logger(),
                    "Action server not available after waiting");
       rclcpp::shutdown();
       return; 
    }
    
    // MoveRobot::Goal()是个结构体，此结构体中只有一个 变量 为distance
    auto goal_msg = MoveRobot::Goal();
    goal_msg.distance = 10;

    RCLCPP_INFO(this->get_logger(), "Sending goal");
    auto send_goal_options = rclcpp_action::Client<MoveRobot>::SendGoalOptions();

    // client中有3个 接受服务端状态 的回调函数,分别是 
     // goal_response: 函数参数为接收1个来自 服务端的 goal_handle，当goal_handle为false时，表示 客户端拒绝
    send_goal_options.goal_response_callback = std::bind(&ActionControl01::goal_response_callback, this, _1);
    
     // feedback: 函数参数为接收1个来自 服务端的 feedback
    send_goal_options.feedback_callback = std::bind(&ActionControl01::feedback_callback, this, _1, _2);
     
     // result: 函数参数为接收1个来自 服务端的 result
    send_goal_options.result_callback = std::bind(&ActionControl01::result_callback, this, _1);

    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

  private:
   rclcpp_action::Client<MoveRobot>::SharedPtr client_ptr_;
   rclcpp::TimerBase::SharedPtr timer_;

   // client's three callback func
    // goal_response_callback，发送目标后得到的响应 的回调函数
   void goal_response_callback(GoalHandleMoveRobot::SharedPtr goal_handle){
     if (!goal_handle){
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
     } else{
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
     }
   }

    // feedback_callback，执行过程中进度反馈 接收回调函数
   void feedback_callback(GoalHandleMoveRobot::SharedPtr, const std::shared_ptr<const MoveRobot::Feedback> feedback){
    RCLCPP_INFO(this->get_logger(), "Feedback current pose:%f", feedback->pose);
   }

    // result_callback，最终结果接收的回调函数。
    void result_callback(const GoalHandleMoveRobot::WrappedResult& result) {
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          break;
        
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
          return;

        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
          return;

        default:
          RCLCPP_ERROR(this->get_logger(), "Unknown result code");
          return;
      }

      RCLCPP_INFO(this->get_logger(), "Result received: %f", result.result->pose);
    }
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  auto action_client = std::make_shared<ActionControl01>("action_robot_cpp");
  rclcpp::spin(action_client);
  rclcpp::shutdown();
  return 0;
}
