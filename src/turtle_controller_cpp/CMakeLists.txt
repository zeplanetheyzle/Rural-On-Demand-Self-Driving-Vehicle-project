cmake_minimum_required(VERSION 3.8)
project(my_turtle_controller_cpp)

# C++ 컴파일 시 경고 옵션을 활성화하여 코드 안정성 향상
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)

# --- 1. draw_square_node 빌드 설정 ---
add_executable(draw_square_node src/draw_square.cpp)
ament_target_dependencies(draw_square_node rclcpp geometry_msgs)
install(TARGETS draw_square_node DESTINATION lib/${PROJECT_NAME})

# --- 2. wall_avoider_node 빌드 설정 (벽 회피) ---
add_executable(wall_avoider_node src/wall_avoider.cpp)
ament_target_dependencies(wall_avoider_node rclcpp geometry_msgs turtlesim)
install(TARGETS wall_avoider_node DESTINATION lib/${PROJECT_NAME})

# --- 3. turtle_stopper_node 빌드 설정 (단순 멈춤 로직) ---
# stop_node.cpp를 stop_node라는 실행 파일로 등록합니다.
add_executable(stop_node src/stop_node.cpp)
ament_target_dependencies(stop_node rclcpp geometry_msgs turtlesim)
install(TARGETS stop_node DESTINATION lib/${PROJECT_NAME})

# 패키지 정보 파일 설치
install(FILES package.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
