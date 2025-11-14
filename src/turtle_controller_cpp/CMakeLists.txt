cmake_minimum_required(VERSION 3.8)
project(my_turtle_controller_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
# [추가!] turtlesim 패키지 찾기
find_package(turtlesim REQUIRED)

# --- 기존 draw_square_node 빌드 설정 (수정 없음) ---
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

# --- [새로 추가!] wall_avoider_node 빌드 설정 ---
add_executable(
  wall_avoider_node
  src/wall_avoider.cpp
)
ament_target_dependencies(
  wall_avoider_node
  rclcpp
  geometry_msgs
  turtlesim # Pose 메시지를 사용하므로 링크
)
install(
  TARGETS wall_avoider_node
  DESTINATION lib/${PROJECT_NAME}
)
# --- 여기까지 추가 ---

install(
  FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
