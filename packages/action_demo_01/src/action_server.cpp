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
    RCLCPP_INFO(this->get_logger(),"接收到任务取消请求;");
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  void handle_accepted(std::shared_ptr<rclcpp_action::ServerGoalHandle<Progress>> goal_handle)
  {
  }
};

int main(int argc, char const *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ActionServer>());
  rclcpp::shutdown();
  return 0;
}
