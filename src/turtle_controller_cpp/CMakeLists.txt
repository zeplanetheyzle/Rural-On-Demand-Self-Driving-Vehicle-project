cmake_minimum_required(VERSION 3.8)
project(my_turtle_controller_cpp)

# C++ 컴파일 시 경고 옵션을 활성화하여 코드 안정성 향상
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED) # 벽 감지에 필요한 Pose 메시지를 위해 필수

# --- 1. draw_square_node 빌드 설정 ---
add_executable(
  draw_square_node
  src/draw_square.cpp
)
ament_target_dependencies(
  draw_square_node
  rclcpp
  geometry_msgs
)
install(
  TARGETS draw_square_node
  DESTINATION lib/${PROJECT_NAME}
)

# --- 2. wall_avoider_node (장애물 멈춤) 빌드 설정 ---
add_executable(
  wall_avoider_node
  src/wall_avoider.cpp
)
ament_target_dependencies(
  wall_avoider_node
  rclcpp
  geometry_msgs
  turtlesim # Pose 메시지 사용
)
install(
  TARGETS wall_avoider_node
  DESTINATION lib/${PROJECT_NAME}
)

# 패키지 정보 파일 설치
install(
  FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

# 테스트 설정 (선택 사항)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# 의존성 내보내기 및 패키지 마무리
ament_package()
