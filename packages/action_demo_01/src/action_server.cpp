#include "rclcpp/rclcpp.hpp"

class ActionServer : public rclcpp::Node
{
private:
public:
  ActionServer() : Node("action_server_node")
  {
    RCLCPP_INFO(this->get_logger(), "动作服务端创建!");
  }
};

int main(int argc, char const *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ActionServer>());
  rclcpp::shutdown();
  return 0;
}
