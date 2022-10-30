#include "rclcpp/rclcpp.hpp"

class ParamServer : public rclcpp::Node
{
public:
  ParamServer() : Node("param_server_node", rclcpp::NodeOptions().allow_undeclared_parameters(true))
  {
    RCLCPP_INFO(this->get_logger(), "参数服务器创建");
  }
  void declare_param()
  {
    RCLCPP_INFO(this->get_logger(), "----------------declare param----------------");
    this->declare_parameter("car_name", "tiger");
    this->declare_parameter("width", 1.55);
    this->declare_parameter("wheels", 5);
    // set_parameter也可以用于设置新参数, 但必须保证rclcpp::NodeOptions().allow_undeclared_parameters(true)
    this->set_parameter(rclcpp::Parameter("height",2.00));
  }
  void get_param()
  {
    RCLCPP_INFO(this->get_logger(), "----------------get param----------------");
    // 获取指定参数
    auto car_name = this->get_parameter("car_name");
    RCLCPP_INFO(this->get_logger(), "key=%s, value=%s,", car_name.get_name().c_str(), car_name.as_string().c_str());
    // 批量获取
    auto params = this->get_parameters({"car_name", "width", "wheels"});
    for (auto &&param : params)
    {
      RCLCPP_INFO(this->get_logger(), "key=%s, value=%s,", param.get_name().c_str(), param.value_to_string().c_str());
    }
    // 判断是否包含
    RCLCPP_INFO(this->get_logger(), "是否包含car_name? %d", this->has_parameter("car_name"));
    RCLCPP_INFO(this->get_logger(), "是否包含height? %d", this->has_parameter("height"));
  }
  void update_param()
  {
    RCLCPP_INFO(this->get_logger(), "----------------update param----------------");
    this->set_parameter(rclcpp::Parameter("width",1.75));
    RCLCPP_INFO(this->get_logger(), "width=%.2f",this->get_parameter("width").as_double());
  }
  void del_param()
  {
    RCLCPP_INFO(this->get_logger(), "----------------del param----------------");
    // this->undeclare_parameter("car_name"); // 不能删除声明的参数
    RCLCPP_INFO(this->get_logger(), "删除前还包含height吗? %d", this->has_parameter("height"));
    this->undeclare_parameter("height");
    RCLCPP_INFO(this->get_logger(), "删除后还包含height吗? %d", this->has_parameter("height"));
    
  }
};

int main(int argc, char const *argv[])
{
  rclcpp::init(argc, argv);
  auto server = std::make_shared<ParamServer>();
  server->declare_param();
  server->get_param();
  server->update_param();
  server->del_param();
  rclcpp::spin(server);
  rclcpp::shutdown();
  return 0;
}
