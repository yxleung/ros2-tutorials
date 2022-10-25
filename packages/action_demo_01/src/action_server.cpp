#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "interface_demo_01/action/progress.hpp"
#include <stdlib.h>

using interface_demo_01::action::Progress;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

class ActionServer : public rclcpp::Node
{
private:
  rclcpp_action::Server<Progress>::SharedPtr server;

public:
  ActionServer() : Node("action_server_node")
  {
    RCLCPP_INFO(this->get_logger(), "动作服务端创建!");
    this->server = rclcpp_action::create_server<Progress>(this, "get_sum", std::bind(&ActionServer::handle_goal, this, _1, _2), std::bind(&ActionServer::handle_cancel, this, _1), std::bind(&ActionServer::handle_accepted, this, _1));
  }
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const Progress::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(),"uuid=%s", rclcpp_action::to_string(uuid).c_str());
    if(goal->num<=1){
      RCLCPP_INFO(this->get_logger(),"提交的目标值必须大于0;");
      return rclcpp_action::GoalResponse::REJECT;
    }
    
    RCLCPP_INFO(this->get_logger(),"提交的目标值合法;");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(std::shared_ptr<rclcpp_action::ServerGoalHandle<Progress>> goal_handle)
  {
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(),"接收到任务取消请求;");
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  void handle_accepted(std::shared_ptr<rclcpp_action::ServerGoalHandle<Progress>> goal_handle)
  {
    (void)goal_handle;
    std::thread(std::bind(&ActionServer::execute,this,goal_handle)).detach();
  }
  void execute(std::shared_ptr<rclcpp_action::ServerGoalHandle<Progress>> goal_handle){
    // 1. 生成连续反馈返回给客户端；
    // 首先要获取目标值；
    int num = goal_handle->get_goal()->num;
    int sum = 0;
    auto feedback = std::make_shared<Progress::Feedback>();
    auto result = std::make_shared<Progress::Result>();
    // 设置休眠
    rclcpp::Rate rate(1.0);
    for(int i=1;i<=num;i++){
      sum += i;
      double progress = i/(double)num;
      feedback->progress = progress;
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(),"连续反馈中，进度:%.2f",progress);
      // 判断是否接受到取消请求
      if(goal_handle->is_canceling()){
        // 如果接受到取消请求,终止程序
        result->sum = sum;
        goal_handle->canceled(result); 
        RCLCPP_INFO(this->get_logger(),"任务被取消了");
        return;
      }

      rate.sleep();
    }
    // 2. 生成最终响应结果;

    if(rclcpp::ok()){
      result->sum = sum;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(),"最终结果:%d",sum);
    }
  }
};

int main(int argc, char const *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ActionServer>());
  rclcpp::shutdown();
  return 0;
}
