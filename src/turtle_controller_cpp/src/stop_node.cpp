/*
 * stop_on_wall.cpp
 * 기능: LiDAR 센서 데이터를 사용하여 전방에 장애물이 일정 거리(30cm) 이내에 감지되면
 * 로봇의 움직임을 즉시 정지시키는 ROS 2(RCLCPP) 노드
 * 환경: TurtleBot3 (Waffle/Burger) + Gazebo 시뮬레이션 환경을 가정
 */

#include "rclcpp/rclcpp.hpp" // ROS 2 C++ 클라이언트 라이브러리
#include "sensor_msgs/msg/laser_scan.hpp" // LiDAR 센서 데이터 메시지 헤더 (거리 정보)
#include "geometry_msgs/msg/twist.hpp" // 로봇 속도 제어 메시지 (선속도/각속도)
#include <memory> // std::shared_ptr, std::make_shared 사용

// 'rclcpp::Node'를 상속받아 'StopOnWall' 클래스를 정의
class StopOnWall : public rclcpp::Node
{
public:
    // 노드 생성자
    StopOnWall() : Node("stop_on_wall")
    {
        // 1. Subscriber 생성: LiDAR Scan 데이터 토픽 ('/scan')을 구독
        // 큐 크기 10으로 설정. 데이터 수신 시 'scanCallback' 함수가 호출
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&StopOnWall::scanCallback, this, std::placeholders::_1));

        // 2. Publisher 생성: 로봇 속도 제어 토픽 ('/cmd_vel')으로 Twist 메시지를 발행
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // 노드 시작 정보 로깅
        RCLCPP_INFO(this->get_logger(), "Stop node for TurtleBot3 started. Threshold: 0.3m.");
    }

private:
    // [LiDAR 데이터 콜백 함수]
    // LiDAR 센서 데이터(LaserScan)가 수신될 때마다 호출됩니다.
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // 1. 전방 거리 측정 (단순화를 위해 스캔 데이터 배열의 정중앙 인덱스만 사용)
        float min_dist = msg->ranges[msg->ranges.size() / 2]; 

        geometry_msgs::msg::Twist cmd; // 로봇에 보낼 속도 명령 메시지 초기화

        // 2. [판단 로직] 충돌 임계값 확인 (30cm = 0.3m)
        if (min_dist < 0.3)  
        {
            // [상황] 장애물이 매우 가까움 (30cm 이내)
            RCLCPP_WARN(this->get_logger(), "Obstacle detected in front! Stopping robot.");
            // [제어] 로봇 정지 명령
            cmd.linear.x = 0.0; // 선속도 0
            cmd.angular.z = 0.0; // 각속도 0
        }
        else
        {
            // [상황] 장애물이 충분히 멀리 있음 (30cm 초과)
            // [제어] 천천히 직진 명령
            cmd.linear.x = 0.15; // 안정적인 저속 직진 (예: 0.15 m/s)
            cmd.angular.z = 0.0;
        }

        // 3. 최종 속도 명령을 'cmd_vel' 토픽으로 발행
        cmd_pub_->publish(cmd);
    }

    // --- 멤버 변수 선언 ---
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_; // LiDAR 데이터 구독자
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_; // 속도 명령 발행자
};

// --- C++ 프로그램의 시작점 ---
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv); // ROS 2 클라이언트 라이브러리 초기화
    // StopOnWall 노드 인스턴스를 생성하고 이벤트 루프를 시작 (콜백 함수들이 실행 대기)
    rclcpp::spin(std::make_shared<StopOnWall>()); 
    rclcpp::shutdown(); // 노드 종료 시 ROS 2 시스템 종료
    return 0;
}
