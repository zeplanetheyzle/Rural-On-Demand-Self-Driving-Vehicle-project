cmake_minimum_required(VERSION 3.8)
project(my_turtle_controller_cpp)

# 컴파일 옵션
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- 필수 패키지 찾기 ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)  # LiDAR LaserScan 메시지

# --- 1. draw_square_node 빌드 ---
add_executable(draw_square_node src/draw_square.cpp)
ament_target_dependencies(draw_square_node rclcpp geometry_msgs)
install(TARGETS draw_square_node
  DESTINATION lib/${PROJECT_NAME}
)

# --- 2. wall_avoider_node 빌드 ---
add_executable(wall_avoider_node src/wall_avoider.cpp)
ament_target_dependencies(wall_avoider_node rclcpp geometry_msgs sensor_msgs)
install(TARGETS wall_avoider_node
  DESTINATION lib/${PROJECT_NAME}
)

# --- 3. stop_node 빌드 ---
add_executable(stop_node src/stop_node.cpp)
ament_target_dependencies(stop_node rclcpp geometry_msgs)
install(TARGETS stop_node
  DESTINATION lib/${PROJECT_NAME}
)

# --- launch 파일 설치 ---
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  PATTERN "*.launch.py"
  PATTERN ".gitkeep" EXCLUDE
)

# 패키지 정보 파일 설치
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

# --- 테스트 ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
