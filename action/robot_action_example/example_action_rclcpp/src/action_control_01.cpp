# include <mutex>
# include <condition_variable>
# include <chrono>

# include "rclcpp/rclcpp.hpp"
# include "rclcpp_action/rclcpp_action.hpp"
# include "robot_control_interfaces/action/move_robot.hpp"

class ActionControl01 : public rclcpp::Node {
 public:
  // action 接口
  using MoveRobot = robot_control_interfaces::action::MoveRobot;
  
  // 客户端与目标交互的类，该类的对象不必使用者刻意创造
  // A Client will create an instance and return it to the user (via a future) after calling Client::async_send_goal.
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
    goal_msg.distance = 15;

    RCLCPP_INFO(this->get_logger(), "Sending goal");
    auto send_goal_options = rclcpp_action::Client<MoveRobot>::SendGoalOptions();

    // client中有3个 接受服务端状态 的回调函数,分别是 
     // goal_response: 函数参数为接收1个来自 服务端的 goal_handle，当goal_handle为false时，表示 客户端拒绝
    send_goal_options.goal_response_callback = std::bind(&ActionControl01::goal_response_callback, this, _1);
    
     // feedback: 函数参数为接收1个来自 服务端的 feedback
    send_goal_options.feedback_callback = std::bind(&ActionControl01::feedback_callback, this, _1, _2);
     
     // result: 函数参数为接收1个来自 服务端的 result
    send_goal_options.result_callback = std::bind(&ActionControl01::result_callback, this, _1);
    
    // std::shared_future<typename GoalHandle::SharedPtr> rclcpp_action::Client< ActionT >::async_send_goal(
    //      const Goal & 	goal,
    //      const SendGoalOptions & 	options = SendGoalOptions()
    // )
    // 如果目标被action server接受，async_send_goal 返回一个包含ClientGoalHandle类的future对象；
    // 如果目标被action server拒绝，async_send_goal 返回 空指针。
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

  void cancel_goal() {
    auto server_ready = client_ptr_->wait_for_action_server(std::chrono::seconds(5));
    if (!server_ready) {
      RCLCPP_ERROR(this->get_logger(),"Action server is not available.");
      return ;
    }

    try {
      // 判断future_cancel的结果和等待
      std::unique_lock<std::mutex> lock(cancel_goal_mutex_);
      RCLCPP_INFO(this->get_logger(),"CancelGoal: run async_cancel_goal request");
      if (client_goal_handle_ != nullptr){
      auto future_cancel = client_ptr_->async_cancel_goal(client_goal_handle_);
      } else{
        RCLCPP_INFO(this->get_logger(),"client_goal_handle_ is nullptr");
        return;
      }
      if (cancel_goal_cv_.wait_for(lock, std::chrono::seconds(9)) == std::cv_status::timeout) {
        RCLCPP_INFO(this->get_logger(),"Get cancel_goal_cv_ value: false");
        cancel_goal_result_ = false;
      } else {
        cancel_goal_result_ = true;
        RCLCPP_INFO(this->get_logger(),"Get cancel_goal_cv_ value: true");
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(),"%s", e.what());
    }
  }

  private:
   std::mutex cancel_goal_mutex_;
   std::condition_variable cancel_goal_cv_;
   bool cancel_goal_result_{true};

   rclcpp_action::Client<MoveRobot>::SharedPtr client_ptr_;
   rclcpp::TimerBase::SharedPtr timer_;
   GoalHandleMoveRobot::SharedPtr client_goal_handle_{nullptr};

   // client's three callback func
    // goal_response_callback，发送目标后得到的响应 的回调函数
   void goal_response_callback(GoalHandleMoveRobot::SharedPtr goal_handle){
   if (!goal_handle){
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
     } else{
      client_goal_handle_ = goal_handle;
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
          break;

        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
          cancel_goal_cv_.notify_one();
          break;

        default:
          RCLCPP_ERROR(this->get_logger(), "Unknown result code");
          break;
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
