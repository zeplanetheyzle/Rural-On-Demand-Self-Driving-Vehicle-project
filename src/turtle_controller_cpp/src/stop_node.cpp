#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

class StopOnWall : public rclcpp::Node
{
public:
    StopOnWall() : Node("stop_on_wall")
    {
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&StopOnWall::scanCallback, this, std::placeholders::_1));

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        RCLCPP_INFO(this->get_logger(), "Stop node for TurtleBot3 started.");
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        float min_dist = msg->ranges[msg->ranges.size() / 2]; // 정면 거리

        geometry_msgs::msg::Twist cmd;

        if (min_dist < 0.3)  // 30cm 이내면 멈춤
        {
            RCLCPP_WARN(this->get_logger(), "Obstacle detected! Stopping.");
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
        }
        else
        {
            cmd.linear.x = 0.15; // 천천히 앞으로
        }

        cmd_pub_->publish(cmd);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StopOnWall>());
    rclcpp::shutdown();
    return 0;
}
