#include "rclcpp/rclcpp.hpp"
#include "interface_demo_01/msg/student.hpp"

using interface_demo_01::msg::Student;
using namespace std::chrono_literals;

class Talker : public rclcpp::Node
{
private:
    rclcpp::Publisher<Student>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    void on_timer()
    {
        auto stu = new Student();
        stu->age = 18;
        stu->name = "lyx";
        stu->heigh = 177.0;
        publisher_->publish(*stu);
    }

public:
    Talker() : Node("talker_node_stu")
    {
        RCLCPP_INFO(this->get_logger(), "发布者创建!");
        publisher_ = this->create_publisher<Student>("chatter_stu", 10);
        timer_ = this->create_wall_timer(500ms, std::bind(&Talker::on_timer, this));
    }
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Talker>());
    rclcpp::shutdown();
    return 0;
}
