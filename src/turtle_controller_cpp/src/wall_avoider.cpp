/*
 * wall_avoider.cpp (TurtleBot3 Gazebo 환경용 수정본)
 * 기능: LiDAR 센서 데이터를 사용하여 장애물을 감지하고 회피합니다.
 */

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp" // [수정] LiDAR 센서 데이터 헤더 추가
#include <memory>
#include <cmath>
#include <algorithm> // std::min, std::max 사용을 위해 추가

using std::placeholders::_1;

// 'rclcpp::Node'를 상속받아 'WallAvoiderNode' 클래스를 정의합니다.
class WallAvoiderNode : public rclcpp::Node
{
public:
    // 노드 생성자
    WallAvoiderNode() : Node("wall_avoider_node"), state_(State::MOVING_STRAIGHT)
    {
        // 1. Publisher 생성: TurtleBot3 표준 제어 토픽 사용
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10); 

        // 2. Subscriber 생성: LiDAR Scan 데이터 구독
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&WallAvoiderNode::scan_callback, this, _1)); 

        // 3. Timer 생성 (메인 컨트롤 루프)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&WallAvoiderNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "'wall_avoider_node' (TurtleBot3 LiDAR 기반)가 시작되었습니다.");
    }

private:
    // 거북이의 현재 상태 (직진 중인지, 회전 중인지)
    enum class State
    {
        MOVING_STRAIGHT,
        TURNING
    };

    // [수정된 scan_callback]
    // 판단 로직을 제거하고, 오직 'latest_scan_' 변수에 최신 LiDAR 데이터를 저장만 합니다.
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        latest_scan_ = *msg;
    }

    // [수정된 timer_callback]
    // 0.1초마다 호출되는 메인 제어 루프. 모든 '판단'과 '제어'를 여기서 수행합니다.
    void timer_callback()
    {
        auto twist_msg = geometry_msgs::msg::Twist(); 

        // 1. 유효성 검사 및 전방 최소 거리 계산
        if (latest_scan_.ranges.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "LiDAR 데이터 수신 대기 중...");
            return;
        }

        // [핵심 수정] LiDAR 데이터 기반 충돌 판단 로직
        // 전방 중앙 (예: -30도에서 +30도) 범위 내의 최소 거리를 찾습니다.
        const double COLLISION_THRESHOLD = 0.8; // 0.8 미터 이내 접근 시 회피 시작
        const double FRONT_ANGLE_RANGE = M_PI / 6.0; // 전방 30도 (0.52 rad)
        
        // TurtleBot3의 LiDAR 데이터는 보통 0도(정면)를 중앙에 둡니다.
        const int num_readings = latest_scan_.ranges.size();
        const int center_index = num_readings / 2;
        
        // 전방 30도에 해당하는 데이터 인덱스를 계산 (TurtleBot3는 360도 스캔 가정)
        // 스캔 범위가 [0, 2*PI] 이므로, 중앙(0도) 주변만 확인합니다.
        const int indices_to_check = static_cast<int>((FRONT_ANGLE_RANGE / latest_scan_.angle_increment));
        
        float min_front_distance = latest_scan_.range_max;

        for (int i = 0; i < num_readings; ++i) {
            // 현재 데이터 포인트의 각도 계산
            double angle = latest_scan_.angle_min + i * latest_scan_.angle_increment;
            // 각도를 -PI에서 PI 범위로 정규화
            if (angle > M_PI) angle -= 2 * M_PI;
            
            // 전방 범위 (-30도 ~ 30도) 내의 유효한 데이터만 확인
            if (std::abs(angle) <= FRONT_ANGLE_RANGE && 
                latest_scan_.ranges[i] > latest_scan_.range_min && 
                latest_scan_.ranges[i] < latest_scan_.range_max) 
            {
                min_front_distance = std::min(min_front_distance, latest_scan_.ranges[i]);
            }
        }
        
        // 2. [판단 로직] 충돌 임계값과 비교
        bool too_close_to_wall = (min_front_distance < COLLISION_THRESHOLD);


        // --- 상태 머신 로직 (turtlesim과 동일하게 유지) ---
        if (state_ == State::MOVING_STRAIGHT)
        {
            if (too_close_to_wall)
            {
                // 1. [판단] 벽/장애물 감지!
                RCLCPP_WARN(this->get_logger(), "장애물 감지! (거리: %.2fm). 회전을 시작합니다.", min_front_distance);
                state_ = State::TURNING; // 2. [상태 변경] '회전'으로 변경
                turn_start_time_ = this->get_clock()->now(); // 3. 회전 시작 시간 기록

                // 4. [제어] 즉시 회전 명령
                twist_msg.linear.x = 0.0;
                twist_msg.angular.z = 0.5; // 느린 회전을 통해 Gazebo 환경 안정성 확보
            }
            else
            {
                // [판단] 장애물 없음 -> [제어] 계속 직진
                twist_msg.linear.x = 0.2; // TurtleBot3의 안정적인 속도 (turtlesim의 1.0보다 느리게)
                twist_msg.angular.z = 0.0;
            }
        }
        else // state_ == State::TURNING
        {
            // [회전 상태]
            // 1. [판단] 2초가 지났는지 확인 (TurtleBot3는 더 무거우므로 회전 시간 증가)
            if ((this->get_clock()->now() - turn_start_time_).seconds() > 2.0)
            {
                RCLCPP_INFO(this->get_logger(), "회전 완료. 다시 직진을 시작합니다.");
                state_ = State::MOVING_STRAIGHT; // 2. [상태 변경] '직진'으로 변경
                
                // 3. [제어] 즉시 직진 명령
                twist_msg.linear.x = 0.2;
                twist_msg.angular.z = 0.0;
            }
            else
            {
                // 1. [판단] 2초가 안 지났다 -> [제어] 계속 회전
                twist_msg.linear.x = 0.0;
                twist_msg.angular.z = 0.5;
            }
        }
        
        // 최종적으로 계산된 속도 명령을 발행(Publish)합니다.
        publisher_->publish(twist_msg);
    }

    // --- 멤버 변수 선언 ---
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_; // [수정] LiDAR 구독
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::LaserScan latest_scan_; // [수정] 최신 LiDAR 데이터 저장
    State state_; 
    rclcpp::Time turn_start_time_{0, 0, this->get_clock()->get_clock_type()}; 
};

// --- C++ 프로그램의 시작점 ---
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallAvoiderNode>());
    rclcpp::shutdown();
    return 0;
}
