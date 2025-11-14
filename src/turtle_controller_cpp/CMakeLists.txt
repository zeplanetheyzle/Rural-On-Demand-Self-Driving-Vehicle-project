cmake_minimum_required(VERSION 3.8)
project(my_turtle_controller_cpp)

# C++ 컴파일 시 경고 옵션을 활성화하여 코드 안정성 향상
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- 필수 패키지 찾기 ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)  # [추가] LiDAR (LaserScan) 메시지 사용을 위해 필수
# find_package(turtlesim REQUIRED)  # [삭제] Turtlesim 의존성 제거

# --- 1. draw_square_node 빌드 설정 (필요 시) ---
# turtlesim 의존성만 제거하고 유지
add_executable(draw_square_node src/draw_square.cpp)
ament_target_dependencies(draw_square_node rclcpp geometry_msgs)
install(TARGETS draw_square_node DESTINATION lib/${PROJECT_NAME})

# --- 2. wall_avoider_node 빌드 설정 (핵심) ---
# [수정] turtlesim 대신 sensor_msgs 의존성 추가
add_executable(wall_avoider_node src/wall_avoider.cpp)
ament_target_dependencies(wall_avoider_node rclcpp geometry_msgs sensor_msgs) 
install(TARGETS wall_avoider_node DESTINATION lib/${PROJECT_NAME})

# --- 3. turtle_stopper_node 빌드 설정 (필요 시) ---
# turtlesim 의존성만 제거하고 유지
add_executable(stop_node src/stop_node.cpp)
ament_target_dependencies(stop_node rclcpp geometry_msgs) 
install(TARGETS stop_node DESTINATION lib/${PROJECT_NAME})

# --- 4. 설치 (install) ---

# 패키지 정보 파일 설치
install(FILES package.xml DESTINATION share/${PROJECT_NAME})

# [추가] launch 디렉토리의 모든 *.launch.py 파일을 설치합니다.
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  PATTERN "*.launch.py"
  PATTERN ".gitkeep" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
