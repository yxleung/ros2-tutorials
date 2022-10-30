#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class ParamClient : public rclcpp::Node
{
private:
    rclcpp::SyncParametersClient::SharedPtr client;

public:
    ParamClient() : Node("param_client_node")
    {
        RCLCPP_INFO(this->get_logger(), "参数客户端创建");
        client = std::make_shared<rclcpp::SyncParametersClient>(this, "param_server_node");
    }

    bool connect_server()
    {
        while (!client->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "连接服务中");
        }
        return true;
    }

    void get_param()
    {
        RCLCPP_INFO(this->get_logger(), "-----------------------参数查询操作-----------------------");
        // 获取指定参数
        auto car_name = client->get_parameter<std::string>("car_name");
        auto width = client->get_parameter<double>("width");
        auto wheels = client->get_parameter<int>("wheels");
        RCLCPP_INFO(this->get_logger(), "car_name=%s, width=%.2f, wheels=%d", car_name.c_str(), width, wheels);
        // 获取批量参数
        auto params = client->get_parameters({"car_name", "width", "wheels"});
        for (auto &&param : params)
        {
            RCLCPP_INFO(this->get_logger(), "k=%s, val=%s", param.get_name().c_str(), param.value_to_string().c_str());
        }
        // 判断是否存在
        RCLCPP_INFO(this->get_logger(), "是否存在参数car_name?%d", client->has_parameter("car_name"));
        RCLCPP_INFO(this->get_logger(), "是否存在参数height?%d", client->has_parameter("height"));
    }

    void update_param()
    {
        RCLCPP_INFO(this->get_logger(), "-----------------------参数更新操作-----------------------");

        client->set_parameters({
            rclcpp::Parameter("car_name","grecole"),
            // 设置服务端不存在的参数
            // 注意：如果允许客户端设置服务端不存在的参数，服务端必须设置：rclcpp::NodeOptions().allow_undeclared_parameters(true)
            rclcpp::Parameter("length", 3)
        });
        RCLCPP_INFO(this->get_logger(), "lenght=%d",client->get_parameter<int>("length"));
    }
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto client = std::make_shared<ParamClient>();
    if (client->connect_server())
    {
        client->get_param();
        client->update_param();
        client->get_param();
    }
    rclcpp::shutdown();
    return 0;
}
