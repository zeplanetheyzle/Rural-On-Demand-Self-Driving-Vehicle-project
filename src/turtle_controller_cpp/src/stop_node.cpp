#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"

// 경계선 설정 (turtlesim 창의 대략적인 크기)
constexpr double MIN_X = 0.5;
constexpr double MAX_X = 10.5;
constexpr double MIN_Y = 0.5;
constexpr double MAX_Y = 10.5;
constexpr double BOUNDARY_THRESHOLD = 1.0; // 벽과의 최소 안전 거리

class TurtleStopper : public rclcpp::Node {
public:
    TurtleStopper() : Node("turtle_stopper_node") {
        // 1. 속도 명령 발행자 (Publisher) 설정
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        
        // 2. 터틀 위치 구독자 (Subscriber) 설정
        subscriber_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 10,
            std::bind(&TurtleStopper::pose_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Turtle Stopper C++ Node has been started.");
    }

private:
    void pose_callback(const turtlesim::msg::Pose::SharedPtr msg) {
        // 터틀의 현재 위치
        double current_x = msg->x;
        double current_y = msg->y;

        // 속도 메시지 초기화 (기본은 멈춤)
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.linear.x = 0.0;
        twist_msg.angular.z = 0.0;

        bool obstacle_detected = false;

        // --- [장애물 감지 로직] ---
        
        // x축 경계 감지
        if (current_x < (MIN_X + BOUNDARY_THRESHOLD) || 
            current_x > (MAX_X - BOUNDARY_THRESHOLD)) {
            
            obstacle_detected = true;
            RCLCPP_WARN(this->get_logger(), "Obstacle detected on X-axis at X=%.2f! Stopping.", current_x);
        }

        // y축 경계 감지
        if (current_y < (MIN_Y + BOUNDARY_THRESHOLD) || 
            current_y > (MAX_Y - BOUNDARY_THRESHOLD)) {
            
            obstacle_detected = true;
            RCLCPP_WARN(this->get_logger(), "Obstacle detected on Y-axis at Y=%.2f! Stopping.", current_y);
        }

        // 장애물이 감지되면 멈춤 명령 발행
        if (obstacle_detected) {
            publisher_->publish(twist_msg); // linear.x와 angular.z는 0.0
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscriber_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    // 노드를 실행하고 메시지 처리를 기다림
    rclcpp::spin(std::make_shared<TurtleStopper>());
    rclcpp::shutdown();
    return 0;
}
