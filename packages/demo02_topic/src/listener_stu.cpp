#include "rclcpp/rclcpp.hpp"
#include "demo01_interface/msg/student.hpp"

using demo01_interface::msg::Student;

class Listener:public rclcpp::Node{
    private:
        rclcpp::Subscription<Student>::SharedPtr subscription_;
        void callback(const Student &stu){
            RCLCPP_INFO(this->get_logger(), "name:%s  , age:%d   ,height:%.2f ;",stu.name.c_str(),stu.age,stu.heigh);
        }
    public:
        Listener():Node("listener_node_stu"){
            RCLCPP_INFO(this->get_logger(),"订阅者创建!");
            subscription_ = this->create_subscription<Student>("chatter_stu",10,std::bind(&Listener::callback,this,std::placeholders::_1));

        }
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<Listener>());
    rclcpp::shutdown();
    return 0;
}
