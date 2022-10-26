#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "interface_demo_01/action/progress.hpp"

using interface_demo_01::action::Progress;
using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

class ActionClient : public rclcpp::Node
{
private:
  rclcpp_action::Client<Progress>::SharedPtr client;
public:
  ActionClient() : Node("action_client_node")
  {
    RCLCPP_INFO(this->get_logger(), "动作客户端创建!");
    client = rclcpp_action::create_client<Progress>(this,"get_sum");
  }
  void send_goal(int num){
   //1. 连接服务端
   if(!client->wait_for_action_server(10s)){
    RCLCPP_ERROR(this->get_logger(),"连接失败");
    return;
   }
   auto goal = Progress::Goal();
   goal.num = num;
   rclcpp_action::Client<Progress>::SendGoalOptions options;
   options.goal_response_callback = std::bind(&ActionClient::goal_response_callback,this,_1);
   options.feedback_callback = std::bind(&ActionClient::feedback_callback,this,_1,_2);
   options.result_callback = std::bind(&ActionClient::result_callback,this,_1);
   //2. 发送请求 
  auto future = client->async_send_goal(goal,options);
  }
  void goal_response_callback(rclcpp_action::ClientGoalHandle<Progress>::SharedPtr goal_handle){
      if(!goal_handle){
        RCLCPP_INFO(this->get_logger(),"目标请求被服务端拒绝");
      }else{
        RCLCPP_INFO(this->get_logger(),"目标处理中");
      }

  }
  void feedback_callback(rclcpp_action::ClientGoalHandle<Progress>::SharedPtr goal_handle, const std::shared_ptr<const Progress::Feedback> feedback){
    (void)goal_handle;
    double progress = feedback->progress;
    int pro = (int)(progress*100);
    RCLCPP_INFO(this->get_logger(),"当前进度:%d%%",pro);
  }
  void result_callback(const rclcpp_action::ClientGoalHandle<Progress>::WrappedResult & result){
    // 通过状态码判断最终结果状态
    if(rclcpp_action::ResultCode::SUCCEEDED == result.code){
      RCLCPP_INFO(this->get_logger(),"最终结果:%d",result.result->sum);
    }else if(rclcpp_action::ResultCode::ABORTED == result.code){
      RCLCPP_INFO(this->get_logger(),"被中断");
    }else if(rclcpp_action::ResultCode::CANCELED == result.code){
      RCLCPP_INFO(this->get_logger(),"被取消");
    }else{
      RCLCPP_INFO(this->get_logger(),"未知异常");
    }
  }
};

int main(int argc, char const *argv[])
{
  if(argc != 2){
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"请传入一个整数");
    return 1;
  }
  rclcpp::init(argc, argv);
  auto client = std::make_shared<ActionClient>();
  client->send_goal(atoi(argv[1]));
  rclcpp::spin(client);
  rclcpp::shutdown();
  return 0;
}
