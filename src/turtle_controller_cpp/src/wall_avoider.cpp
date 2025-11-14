/*
 * wall_avoider.cpp 
 * 기능: **LiDAR 센서 데이터**를 사용하여 **전방 장애물을 감지**하고, 정해진 시간(2.0초) 동안 회전하여 **단순 벽/장애물 회피**를 수행하는 ROS 2(Foxy/Galactic/Humble 이상) 노드입니다.
 * 환경: TurtleBot3 (Waffle/Burger) + Gazebo 시뮬레이션 환경에 최적화된 속도 설정(linear.x=0.2, angular.z=0.5)을 사용합니다.
 */

#include "rclcpp/rclcpp.hpp" // ROS 2 C++ 클라이언트 라이브러리
#include "geometry_msgs/msg/twist.hpp" // 로봇 속도 제어 메시지 (선속도/각속도)
#include "sensor_msgs/msg/laser_scan.hpp" // LiDAR 센서 데이터 메시지 헤더 추가 (거리 정보)
#include <memory> // std::shared_ptr, std::make_shared 사용
#include <cmath> // 삼각 함수 및 M_PI (파이) 상수 사용
#include <algorithm> // std::min, std::max 사용을 위해 추가 (전방 최소 거리 계산)

using std::placeholders::_1; // std::bind를 위한 플레이스홀더

// 'rclcpp::Node'를 상속받아 'WallAvoiderNode' 클래스를 정의하기
class WallAvoiderNode : public rclcpp::Node
{
public:
    // 노드 생성자
    WallAvoiderNode() : Node("wall_avoider_node"), state_(State::MOVING_STRAIGHT)
    {
        // 1. Publisher 생성: TurtleBot3 표준 제어 토픽 ('/cmd_vel')으로 Twist 메시지를 발행함
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10); 

        // 2. Subscriber 생성: LiDAR Scan 데이터 토픽 ('/scan')을 구독하기
        // 콜백 함수 'scan_callback'이 LiDAR 데이터 수신 시 호출
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&WallAvoiderNode::scan_callback, this, _1)); 

        // 3. Timer 생성 (메인 컨트롤 루프): 0.1초(100ms)마다 'timer_callback' 함수를 호출하여 로봇의 상태 판단 및 제어를 수행
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&WallAvoiderNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "'wall_avoider_node' (TurtleBot3 LiDAR 기반)가 시작되었습니다. 충돌 임계값: 0.8m");
    }

private:
    enum class State
    {
        MOVING_STRAIGHT, // 직진 중: 장애물 감지 여부를 지속적으로 확인하는 상태
        TURNING          // 회전 중: 장애물을 회피하기 위해 정해진 시간 동안 회전하는 상태
    };

    // [LiDAR 데이터 처리 콜백]
    // LiDAR 센서 데이터 메시지 수신 시 호출됩니다.
    // 역할: 판단 로직을 제거하고, 오직 수신된 최신 LiDAR 데이터('msg')를 클래스 멤버 변수 'latest_scan_'에 저장
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        latest_scan_ = *msg; // 메시지의 내용을 멤버 변수에 복사하여 메인 루프에서 사용하도록 준비
    }

    // 0.1초마다 호출되며, 로봇의 모든 '판단'과 '제어' 명령 생성 함수
    void timer_callback()
    {
        auto twist_msg = geometry_msgs::msg::Twist(); // 발행할 속도 메시지 초기화

        // 1. 유효성 검사 및 전방 최소 거리 계산
        if (latest_scan_.ranges.empty()) {
            // LiDAR 데이터가 아직 수신되지 않았을 경우 경고 메시지를 발행하고 함수 종료 (1초에 한 번만 출력)
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "LiDAR 데이터 수신 대기 중...");
            return;
        }

        // --- LiDAR 데이터 기반 충돌 판단 로직 ---
        const double COLLISION_THRESHOLD = 0.8; // **충돌 임계값:** 0.8 미터 이내 접근 시 회피 기동 시작
        const double FRONT_ANGLE_RANGE = M_PI / 6.0; // **전방 감지 범위:** 정면에서 좌우 30도 (총 60도, 약 0.52 rad)

        const int num_readings = latest_scan_.ranges.size(); // 총 LiDAR 데이터 포인트 수

        float min_front_distance = latest_scan_.range_max; // 전방 최소 거리를 최대 값으로 초기화

        // LiDAR 스캔 배열을 순회하며 전방 범위 내의 최소 거리를 찾기기
        for (int i = 0; i < num_readings; ++i) {
            // 현재 데이터 포인트의 각도 계산 (라디안)
            double angle = latest_scan_.angle_min + i * latest_scan_.angle_increment;
            
            // 각도를 $- \pi$ 에서 $\pi$ 범위로 정규화 (보통 0도가 정면)
            if (angle > M_PI) angle -= 2 * M_PI; 
            
            // 전방 지정 범위($\pm 30^\circ$) 내에 있고 유효한 값(min < range < max)인 경우만 확인
            if (std::abs(angle) <= FRONT_ANGLE_RANGE && 
                latest_scan_.ranges[i] > latest_scan_.range_min && 
                latest_scan_.ranges[i] < latest_scan_.range_max) 
            {
                min_front_distance = std::min(min_front_distance, latest_scan_.ranges[i]);
            }
        }
        
        // 2. [판단 로직] 전방 최소 거리를 충돌 임계값과 비교
        bool too_close_to_wall = (min_front_distance < COLLISION_THRESHOLD);


        // --- 상태 머신 로직 (State Machine) ---
        if (state_ == State::MOVING_STRAIGHT)
        {
            // 현재 상태: 직진 중
            if (too_close_to_wall)
            {
                // 1. [판단] 벽/장애물 감지!
                RCLCPP_WARN(this->get_logger(), "장애물 감지! (거리: %.2fm). 회전을 시작합니다.", min_front_distance);
                state_ = State::TURNING; // 2. [상태 변경] '회전' 상태로 전환
                turn_start_time_ = this->get_clock()->now(); // 3. 회피 회전 시작 시간 기록 (시간 기반 제어)

                // 4. [제어] 즉시 회전 명령 발행 (선속도 0, 좌회전)
                twist_msg.linear.x = 0.0;
                twist_msg.angular.z = 0.5; // Gazebo 환경에서 안정적인 각속도 (느린 회전)
            }
            else
            {
                // [판단] 장애물 없음 -> [제어] 계속 직진
                twist_msg.linear.x = 0.2; // TurtleBot3에 적합한 비교적 느린 선속도
                twist_msg.angular.z = 0.0;
            }
        }
        else // state_ == State::TURNING (회전 상태)
        {
            // [회전 상태]
            // 1. [판단] 회전 시작 시간으로부터 2.0초가 경과했는지 확인 (회전 지속 시간)
            if ((this->get_clock()->now() - turn_start_time_).seconds() > 2.0)
            {
                RCLCPP_INFO(this->get_logger(), "회전 완료. 다시 직진을 시작합니다.");
                state_ = State::MOVING_STRAIGHT; // 2. [상태 변경] '직진' 상태로 전환
                
                // 3. [제어] 즉시 직진 명령 발행
                twist_msg.linear.x = 0.2;
                twist_msg.angular.z = 0.0;
            }
            else
            {
                // 1. [판단] 2.0초가 아직 안 지났다 -> [제어] 계속 회전 명령 유지
                twist_msg.linear.x = 0.0;
                twist_msg.angular.z = 0.5;
            }
        }
        
        // 최종적으로 계산된 속도 명령 (twist_msg)을 'cmd_vel' 토픽으로 발행
        publisher_->publish(twist_msg);
    }

    // --- 멤버 변수 선언 ---
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_; // 속도 명령 발행을 위한 Publisher
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_; // LiDAR 데이터를 구독하기 위한 Subscriber
    rclcpp::TimerBase::SharedPtr timer_; // 메인 제어 루프를 주기적으로 실행하는 Timer
    sensor_msgs::msg::LaserScan latest_scan_; // 최신 LiDAR 센서 데이터를 저장하는 변수
    State state_; // 현재 로봇의 상태 (직진/회전)
    rclcpp::Time turn_start_time_{0, 0, this->get_clock()->get_clock_type()}; // 회전 시작 시간을 기록하는 변수 (시간 기반 제어용)
};

// --- C++ 프로그램의 시작점 ---
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv); // ROS 2 클라이언트 라이브러리 초기화
    rclcpp::spin(std::make_shared<WallAvoiderNode>()); // WallAvoiderNode 인스턴스를 생성하고 이벤트 루프를 시작 (노드 실행)
    rclcpp::shutdown(); // 노드 종료 시 ROS 2 시스템 종료
    return 0;
}
