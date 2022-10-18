#include "rclcpp/rclcpp.hpp"
#include "interface_demo_01/srv/add_ints.hpp"

using interface_demo_01::srv::AddInts;

class Server : public rclcpp::Node
{
private:
    rclcpp::Service<AddInts>::SharedPtr service_;
    void add(const AddInts::Request::SharedPtr req,const AddInts::Response::SharedPtr resp)
    {
        resp->sum = req->num1 + req->num2;
        RCLCPP_INFO(this->get_logger(), "exec add function: %d = %d + %d", resp->sum, req->num1, req->num2);
    }

public:
    Server() : Node("server_01_node")
    {
        RCLCPP_INFO(this->get_logger(), "服务创建!");
        service_ = this->create_service<AddInts>("add_int", std::bind(&Server::add, this, std::placeholders::_1, std::placeholders::_2));
    }
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Server>());
    rclcpp::shutdown();
    return 0;
}
