/*
 * draw_square.cpp
 * 기능: ROS 2의 rclcpp::Timer와 상태 변수(step_)를 사용하여
 * '직진'과 '90도 회전'을 반복함으로써 로봇이 정사각형 궤적을 그리도록 제어하는 노드
 * 환경: TurtleBot3 (Waffle/Burger) + Gazebo 시뮬레이션 환경을 가정
 * 제어 방식: 시간 기반 제어 (Time-based control)를 사용합니다.
 */

#include "rclcpp/rclcpp.hpp" // ROS 2 C++ 클라이언트 라이브러리
#include "geometry_msgs/msg/twist.hpp" // 로봇 속도 제어 메시지 (선속도/각속도)
#include <chrono> // 시간 관련 라이브러리 (std::chrono_literals 사용)
#include <memory> // std::shared_ptr, std::make_shared 사용
#include <functional> // std::bind 사용

using namespace std::chrono_literals; // 50ms 와 같은 시간 리터럴 사용을 위해 선언

// 'rclcpp::Node'를 상속받아 'DrawSquare' 클래스를 정의
class DrawSquare : public rclcpp::Node
{
public:
    // 노드 생성자
    DrawSquare() : Node("draw_square")
    {
        // 1. Publisher 생성: 로봇 속도 제어 토픽 ('/cmd_vel')으로 Twist 메시지를 발행
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        // 2. Timer 생성: 50ms(0.05초)마다 'loop' 함수를 호출하여 로봇의 상태 업데이트 및 속도 명령을 발행
        timer_ = this->create_wall_timer(50ms, std::bind(&DrawSquare::loop, this));

        // 3. 초기 상태 설정
        start_time_ = this->now(); // 시작 시간 기록
        step_ = 0; // 초기 상태: 0 (직진)

        RCLCPP_INFO(this->get_logger(), "TurtleBot3 draw_square started. Starting straight movement.");
    }

private:

    // 50ms마다 호출되며, 시간과 현재 단계(step_)를 기반으로 로봇의 속도를 결정합니다.
    void loop()
    {
        auto now = this->now(); // 현재 시간 획득
        // 현재 단계(직진/회전)가 시작된 이후 경과된 시간 계산 (초 단위)
        auto elapsed = (now - start_time_).seconds(); 
        geometry_msgs::msg::Twist cmd; // 발행할 속도 메시지 초기화

        // --- 상태 머신 로직 (Switch Case) ---
        switch (step_)
        {
        case 0: // 상태 0: 직진 (정사각형의 한 변 그리기)
            if (elapsed < 2.0)
            {
                // 2.0초 동안 직진 명령 유지 (속도: 0.2 m/s)
                cmd.linear.x = 0.2;
                cmd.angular.z = 0.0;
            }
            else {
                // 직진 시간이 2.0초 경과
                RCLCPP_INFO(this->get_logger(), "Straight movement finished. Starting turn.");
                step_ = 1; // 상태 1 (회전)으로 전환
                start_time_ = now; // 타이머 재설정: 회전 시작 시간을 현재 시간으로 갱신
            }
            break;

        case 1: // 상태 1: 회전 (90도 회전하여 다음 변 준비)
            if (elapsed < 1.6)
            {
                // 1.6초 동안 제자리 회전 명령 유지 (각속도: 0.9 rad/s, 약 90도 회전에 필요한 시간)
                cmd.linear.x = 0.0;
                cmd.angular.z = 0.9;
            }
            else {
                // 회전 시간이 1.6초 경과
                RCLCPP_INFO(this->get_logger(), "Turn finished. Starting next straight movement.");
                step_ = 0; // 상태 0 (직진)으로 복귀 (정사각형의 다음 변 시작)
                start_time_ = now; // 타이머 재설정: 직진 시작 시간을 현재 시간으로 갱신
            }
            break;
        
        // 추가적인 case가 없으므로, 두 상태를 반복하여 정사각형을 계속 그리기
        // 정지하려면 여기에 정지 로직(예: 4번의 직진/회전 후 step_ = 2)을 추가!!
        }

        // 계산된 속도 명령 (cmd)을 '/cmd_vel' 토픽으로 발행
        cmd_pub_->publish(cmd);
    }

    // --- 멤버 변수 선언 ---
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_; // 속도 명령 발행자
    rclcpp::TimerBase::SharedPtr timer_; // 주기적으로 loop() 함수를 실행하는 타이머
    rclcpp::Time start_time_; // 현재 단계(직진 또는 회전)가 시작된 시간을 기록
    int step_; // 현재 로봇의 동작 단계 (0: 직진, 1: 회전)
};

// --- C++ 프로그램의 시작점 ---
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv); // ROS 2 클라이언트 라이브러리 초기화
    // DrawSquare 노드 인스턴스를 생성하고 이벤트 루프를 시작 (Timer가 주기적으로 loop 호출)
    rclcpp::spin(std::make_shared<DrawSquare>()); 
    rclcpp::shutdown(); // 노드 종료 시 ROS 2 시스템 종료
    return 0;
}
