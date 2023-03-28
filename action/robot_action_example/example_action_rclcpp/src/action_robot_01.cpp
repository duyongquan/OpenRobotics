# include "example_action_rclcpp/robot.h"
# include "rclcpp/rclcpp.hpp"
# include "rclcpp_action/rclcpp_action.hpp"
# include "robot_control_interfaces/action/move_robot.hpp"
# include <mutex>
# include <future>
# include <chrono>
# include <thread>

/*
1. server端改为包含两个ServerGoalHandle： current_handle,pending_handle；
2. ActionRobot01改为模板类，可以接受任意 action 接口。
*/

// create a ActionSever Class
// template<typename ActionT>
class ActionRobot01 : public rclcpp::Node {
 public:
  // action 接口
  using MoveRobot = robot_control_interfaces::action::MoveRobot;

  // GoalHandleMoveRobot is a server of a specific action (is a template class)
  using GoalHandleMoveRobot = rclcpp_action::ServerGoalHandle<MoveRobot>;
  
  // 构造函数
  explicit ActionRobot01(std::string name) : Node(name){
    RCLCPP_INFO(this->get_logger(), "node has been started: %s.", name.c_str());

    using namespace std::placeholders;

// template<typename ActionT, typename NodeT>
// Server<ActionT>::SharedPtr rclcpp_action::create_server(NodeT                                              node,
//                                                         const std::string &                                name,
//                                                         typename Server< ActionT >::GoalCallback           handle_goal,
//                                                         typename Server< ActionT >::CancelCallback         handle_cancel,
//                                                         typename Server< ActionT >::AcceptedCallback       handle_accepted,
//                                                         const rcl_action_server_options_t &                options = rcl_action_server_get_default_options(),
//                                                         rclcpp::CallbackGroup::SharedPtr                   group = nullptr
//                                                        )
// three callback func, aimed to deal with 'receive goal', 'receive cancel' and 'make sure accept and execute' 
    this->action_server_ = rclcpp_action::create_server<MoveRobot>(
        this, "move_robot",
        std::bind(&ActionRobot01::handle_goal, this, _1, _2),
        std::bind(&ActionRobot01::handle_cancel, this, _1),
        std::bind(&ActionRobot01::handle_accepted, this, _1));
  }

/**
   * @brief Whether a given goal handle is currently active
   * @param handle Goal handle to check
   * @return Whether this goal handle is active
   */
  bool is_active(const std::shared_ptr<GoalHandleMoveRobot> handle) const
  {
    // handle->is_active() 是 serverGoalHandle的成员函数，返回true表示 任务 正被挂起 或 正在执行
    // 返回false表示 任务 为 终止状态
    return handle != nullptr && handle->is_active();
  }
  
  /**
   * @brief Whether the action server is munching on a goal
   * @return bool If its running or not
   */
  bool is_running()
  {
    // execution_future_状态为 false时， 说明任务正在执行完成 或 未执行完成。 
    // execution_future_状态为 true，说明有任务正在执行中。 
    return execution_future_.valid() &&
           (execution_future_.wait_for(std::chrono::milliseconds(0)) == std::future_status::timeout);
  }


  /**
   * @brief Active action server
   * 用于lifecycle
   */
  void activate()
  {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);
    server_active_ = true;
    stop_execution_ = false;
  }

   /**
   * @brief Deactive action server
   * 用于lifecycle
   */
  void deactivate()
  {
    RCLCPP_INFO(this->get_logger(), "Deactivating...");
    {
      std::lock_guard<std::recursive_mutex> lock(update_mutex_);
      server_active_ = false;
      stop_execution_ = true;
    }

    if (!execution_future_.valid()) {
      return;
    }

    if (is_running()) {
      RCLCPP_INFO(this->get_logger(),
        "Requested to deactivate server but goal is still executing."
        " Should check if action server is running before deactivating.");
    }

    using namespace std::chrono;  //NOLINT
    auto start_time = steady_clock::now();
    while (execution_future_.wait_for(milliseconds(100)) != std::future_status::ready) {
      RCLCPP_INFO(this->get_logger(), "Waiting for async process to finish.");
      if (steady_clock::now() - start_time >= server_timeout_) {
        terminate_all();
        // completion_callback_();
        throw std::runtime_error("Action callback is still running and missed deadline to stop");
      }
    }

    RCLCPP_INFO(this->get_logger(),"Deactivation completed.");
  }

  void terminate(
    std::shared_ptr<GoalHandleMoveRobot> handle,
    typename std::shared_ptr<MoveRobot::Result> result =
    std::make_shared<MoveRobot::Result>())
  {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);

    if (is_active(handle)) {
      // indicate if client has requested this goal be cancelled
      // 鲁棒性考虑
      if (handle->is_canceling()) {
        RCLCPP_INFO(this->get_logger(), "Client requested to cancel the goal. Cancelling.");
        // indicate that a goal has been canceled 
         // 当目标正在执行或挂起 但客户端已发出取消时才能调用此方法，故此函数被嵌套在is_canceling()中
         // para: 最后的 结果 发送到客户端
        handle->canceled(result);
      } else {
        RCLCPP_INFO(this->get_logger(),"Aborting handle.");
        handle->abort(result);
      }
      handle.reset();
      bool active = is_active(handle);
      RCLCPP_INFO(this->get_logger(),"Handle active status is %i after setting", active);
    }
  }

/**
   * @brief Terminate all pending and active actions
   * @param result A result object to send to the terminated actions
   */
  void terminate_all(
    typename std::shared_ptr<MoveRobot::Result> result =
    std::make_shared<MoveRobot::Result>())
  {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);
    terminate(current_handle_, result);
    terminate(pending_handle_, result);
    preempt_requested_ = false;
  }

  private:
   Robot robot;

   rclcpp_action::Server<MoveRobot>::SharedPtr action_server_;
   
   std::future<void> execution_future_;
   bool stop_execution_{false};

   mutable std::recursive_mutex update_mutex_;
   // server_active_本应该是false，在 action_server被封装在一个lifecycleNode中
   // bool server_active_{false};
   bool server_active_{true};
   bool preempt_requested_{false};
   std::chrono::milliseconds server_timeout_;

   std::shared_ptr<GoalHandleMoveRobot> current_handle_;
   std::shared_ptr<GoalHandleMoveRobot> pending_handle_;

   // pass a goal share_ptr to the func,then return 'execute goal'(ACCEPT_AND_EXECUTE) or 'reject goal(REJECT)'
   rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const MoveRobot::Goal> goal){
      
      std::lock_guard<std::recursive_mutex> lock(update_mutex_);
      RCLCPP_INFO(this->get_logger(), "Received goal request with distance %f",
                  goal->distance);

      (void)uuid;
      
      if (!server_active_){
        RCLCPP_INFO(this->get_logger(), "The action server status is deactive, the robot reject the order!");
        return rclcpp_action::GoalResponse::REJECT;
      } else{
        RCLCPP_INFO(this->get_logger(), "The action server status is active, the robot can receive the order.");
        if (fabs(goal->distance > 100)){
          RCLCPP_INFO(this->get_logger(), "The target is too far, the robot reject the order!");
          return rclcpp_action::GoalResponse::REJECT;
        }
      }

      RCLCPP_INFO(this->get_logger(),"The target is %f,the robot can move there.", goal->distance);
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // pass rclcpp_action::ServerGoalHandle<...> share_ptr to the func, 
    // then return ACCEPT or REJECT
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMoveRobot> goal_handle){
      std::lock_guard<std::recursive_mutex> lock(update_mutex_);
      
      if(!goal_handle->is_active()){
        RCLCPP_INFO(this->get_logger(), 
        "Received request for goal cancellation,but the handle is inactive, so reject the request");
      }
       
      RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
      (void)goal_handle;
      robot.stop_move(); 
      return rclcpp_action::CancelResponse::ACCEPT;
    }

    
    void execute_move(const std::shared_ptr<GoalHandleMoveRobot> handle) 
    {
      // get_goal() 是 模板类 rclcpp_action::ServerGoalHandle<MoveRobot> 的 成员函数 
      const auto goal = handle->get_goal();
      RCLCPP_INFO(this->get_logger(), "The robot start move %f ...", goal->distance);

      auto result = std::make_shared<MoveRobot::Result>();
      // Rate函数 来精准控制循环的周期，让其保持为2HZ
      rclcpp::Rate rate = rclcpp::Rate(2);
      robot.set_goal(goal->distance);
      
      // 采用了while循环的形式，不断 调用机器人移动并获取机器人的位置，client自動調用feedback进行反馈。
      // 同时检测任务是否被取消，如顺利执行完成则反馈最终结果。
      // action_client 发送
      // async_cancel_goal(typename GoalHandle::SharedPtr handle, CancelCallback cancel_callback = nullptr)
      // 则任务取消	
      while (rclcpp::ok() && !robot.close_goal()) {
        robot.move_step();

        auto feedback = std::make_shared<MoveRobot::Feedback>();
        feedback->pose = robot.get_current_pose();
        feedback->status = robot.get_status();

        // topic
        // publish_feedback() 是 模板类 rclcpp_action::ServerGoalHandle<MoveRobot> 的 成员函数 
        handle->publish_feedback(feedback);

        /* detect the mission cancel or not */
        // is_canceling() 是 模板类 rclcpp_action::ServerGoalHandle<MoveRobot> 的 成员函数 
        if (handle->is_canceling()) {
          result->pose = robot.get_current_pose();

          handle->canceled(result);
          RCLCPP_INFO(this->get_logger(), "Goal Canceled");
          return;
        }
        RCLCPP_INFO(this->get_logger(), "Publish Feedback"); /*Publish feedback*/
        rate.sleep();
      }

      result->pose = robot.get_current_pose();
      handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal Succeeded");
    }

  // 当goal_handle中对 移动请求 ACCEPT后 则进入到这里进行执行，这里是单独开了个线程进行执行execute_move函数，目的是避免阻塞主线程。
  void handle_accepted(const std::shared_ptr<GoalHandleMoveRobot> goal_handle) 
  {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);
    RCLCPP_INFO(this->get_logger(), "Receiveing a new goal");
    
    if(is_active(current_handle_) || is_running()){

      RCLCPP_INFO(this->get_logger(), 
      "Current_handle_ is busy, Current_handle_ active status and is_running() are %i, %i respectively",
      is_active(current_handle_),is_running()
      );
       
      if (is_active(pending_handle_)) {
        RCLCPP_INFO(this->get_logger(), "Pending_handle_ is also busy, action server have to terminate pending_handle_");
        terminate(pending_handle_);
      }
      // 传进来的新任务为 抢占 pending_handle_
      RCLCPP_INFO(this->get_logger(), "Another goal preempts pending_handle_");
      bool handleIsActive = is_active(pending_handle_);
      RCLCPP_INFO(this->get_logger(),"Pending_handle_ active is %i before instancing", handleIsActive);
      pending_handle_ = goal_handle;
      handleIsActive = is_active(pending_handle_);
      RCLCPP_INFO(this->get_logger(),"Pending_handle_ active is %i after instancing", handleIsActive);

      preempt_requested_ = true;
    } else{
      // current_handle_ 没有执行 任务，也没有任务被挂起
      RCLCPP_INFO(this->get_logger(), "Current_handle_ is idle");
      if (is_active(pending_handle_)) {
        // Shouldn't reach a state with a pending goal but no current one.
        RCLCPP_INFO(this->get_logger(), "Forgot to handle a preemption. Terminating the pending goal.");
        terminate(pending_handle_);
        preempt_requested_ = false;
      }

      // pending_handle_、current_handle_均是 deactive状态
      bool handleIsActive = is_active(current_handle_);
      RCLCPP_INFO(this->get_logger(),"Current_handle_ active is %i before instancing", handleIsActive);
      current_handle_ = goal_handle;
      handleIsActive = is_active(current_handle_);
      RCLCPP_INFO(this->get_logger(),"Current_handle_ active is %i after instancing", handleIsActive);

      // Return quickly to avoid blocking the executor, so spin up a new thread
      RCLCPP_INFO(this->get_logger(),"Executing goal asynchronously.");
      // std::async 将 std::future、 std::promise、 std::packaged_task 三者封装在一起
      // std::packaged_task 中保存了 work()函数 执行的结果

      // 任务一旦开支执行 execution_future_.valid() 返回 true，也即is_running()返回true。
      // 当 execute_move 执行完成后 execution_future_.valid() 变成 false
      // execution_future_在is_running函数中判断
      execution_future_ = std::async(std::launch::async,[this]() {work();});
    }

    // using std::placeholders::_1;
    // std::thread{std::bind(&ActionRobot01::execute_move, this, _1), goal_handle}
    //     .detach();
  }

  void work()
  {
    // stop_execution_仅在lifenode被执行deactive的时候 变为false
    // while (rclcpp::ok() && !stop_execution_ && is_active(current_handle_)) 
    while (rclcpp::ok() && is_active(current_handle_)){
      RCLCPP_INFO(this->get_logger(),"Executing the goal...");
      try {
        execute_move(current_handle_);
        bool handleIsActive = is_active(current_handle_);
        RCLCPP_INFO(this->get_logger(),"Current_handle_ active is %i after execute_move", handleIsActive);
      } catch (std::exception & ex) {
        RCLCPP_INFO(this->get_logger(),
          "Action server failed while executing action callback: \"%s\"", ex.what());
        terminate_all();
        // completion_callback_();
        return;
      }

      RCLCPP_INFO(this->get_logger(),"Blocking processing of new goal handles.");
      std::lock_guard<std::recursive_mutex> lock(update_mutex_);
      
      // 仅在lifenode被执行deactive的时候 变为false
      // if (stop_execution_) {
      //   RCLCPP_INFO(this->get_logger(),"Stopping the thread per request.");
      //   terminate_all();
      //   // completion_callback_();
      //   break;
      // }

      if (is_active(current_handle_)) {
        RCLCPP_INFO(this->get_logger(),"Current goal was not completed successfully.");
        terminate(current_handle_);
        // completion_callback_();
      }

      if (is_active(pending_handle_)) {
        RCLCPP_INFO(this->get_logger(),"Executing a pending handle on the existing thread.");
        accept_pending_goal();
      } else {
        RCLCPP_INFO(this->get_logger(),"Done processing available goals.");
        break;
      }
    }
    RCLCPP_INFO(this->get_logger(),"Worker thread done.");
  }

/**
   * @brief Accept pending goals
   * @return Goal Ptr to the  goal that's going to be accepted
   */
  std::shared_ptr<const MoveRobot::Goal> accept_pending_goal()
  {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);
    
    // 若pending_handle_未实例化 或 pending_handle_没有挂起的任务
    if (!pending_handle_ || !pending_handle_->is_active()) {
      RCLCPP_INFO(this->get_logger(),"Attempting to get pending goal when not available");
      return std::shared_ptr<MoveRobot::Goal>();
    }

    if (is_active(current_handle_) && current_handle_ != pending_handle_) {
      RCLCPP_INFO(this->get_logger(),"Cancelling the previous goal");
      current_handle_->abort(std::make_shared<MoveRobot::Result>());
    }

    current_handle_ = pending_handle_;
    pending_handle_.reset();
    preempt_requested_ = false;

    RCLCPP_INFO(this->get_logger(),"Preempted goal");

    return current_handle_->get_goal();
  }

  // constexpr auto empty_result() const
  // {
  //   return std::make_shared<MoveRobot::Result>();
  // }
};



int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto action_server = std::make_shared<ActionRobot01>("action_robot_01");
    rclcpp::spin(action_server);
    rclcpp::shutdown();
    return 0;
}
