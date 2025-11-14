#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <chrono>

using namespace std::chrono_literals;

class DrawSquare : public rclcpp::Node
{
public:
    DrawSquare() : Node("draw_square")
    {
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        timer_ = this->create_wall_timer(50ms, std::bind(&DrawSquare::loop, this));

        start_time_ = this->now();
        step_ = 0;

        RCLCPP_INFO(this->get_logger(), "TurtleBot3 draw_square started.");
    }

private:
    void loop()
    {
        auto now = this->now();
        auto elapsed = (now - start_time_).seconds();
        geometry_msgs::msg::Twist cmd;

        switch (step_)
        {
        case 0: // 직진 2초
            if (elapsed < 2.0)
                cmd.linear.x = 0.2;
            else {
                step_ = 1;
                start_time_ = now;
            }
            break;

        case 1: // 회전 1.6초 (약 90도)
            if (elapsed < 1.6)
                cmd.angular.z = 0.9;
            else {
                step_ = 0;
                start_time_ = now;
            }
            break;
        }

        cmd_pub_->publish(cmd);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time start_time_;
    int step_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DrawSquare>());
    rclcpp::shutdown();
    return 0;
}
