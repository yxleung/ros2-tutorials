#include "rclcpp/rclcpp.hpp"
#include "interface_demo_01/srv/add_ints.hpp"

using interface_demo_01::srv::AddInts;
using namespace std::chrono_literals;

class Client : public rclcpp::Node
{
private:
    rclcpp::Client<AddInts>::SharedPtr client_;

public:
    Client() : Node("client_01_node")
    {
        RCLCPP_INFO(this->get_logger(), "客户端创建!");
        client_ = this->create_client<AddInts>("add_int");
        // auto req = std::make_shared<AddInts>();
        // req->num1 = 1;
        // req->num2 = 2;
        // client_->async_send_request(req);
    }
    bool connect()
    {
        while (!client_->wait_for_service(1s))
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "正在尝试连接服务!");
            if (!rclcpp::ok())
            {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "程序被中断!");
                return false;
            }
        }

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "服务连接成功!");
        return true;
    }
    rclcpp::Client<AddInts>::FutureAndRequestId send(int num1, int num2)
    {
        auto request = std::make_shared<AddInts::Request>();
        request->num1 = num1;
        request->num2 = num2;
        return client_->async_send_request(request);
    }
};

int main(int argc, char const *argv[])
{
    if (argc != 3)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "请提交两个整型数字!");
        return 1;
    }

    rclcpp::init(argc, argv);
    auto client = std::make_shared<Client>();
    bool flag = client->connect();
    if (!flag)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "服务器连接失败!");
        return 1;
    }
    auto future = client->send(atoi(argv[1]), atoi(argv[2]));

    if (rclcpp::spin_until_future_complete(client, future) == rclcpp::FutureReturnCode::SUCCESS)
    {

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "响应成功! sum=%d", future.get()->sum);
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "响应失败!");
    }

    rclcpp::shutdown();
    return 0;
}
