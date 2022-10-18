#include "rclcpp/rclcpp.hpp"

class ActionClient : public rclcpp::Node
{
private:
public:
  ActionClient() : Node("action_client_node")
  {
    RCLCPP_INFO(this->get_logger(), "动作客户端创建!");
  }
};

int main(int argc, char const *argv[])
{
  rclcpp::init(argc, argv);
  auto client = std::make_shared<ActionClient>();
  rclcpp::shutdown();
  return 0;
}
