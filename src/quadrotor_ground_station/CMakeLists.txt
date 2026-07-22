cmake_minimum_required(VERSION 3.8)
project(quadrotor_ground_station)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(hmi_touch_verify_node src/hmi_touch_verify_node.cpp)
ament_target_dependencies(hmi_touch_verify_node rclcpp)

add_executable(serial_loopback_test_node src/serial_loopback_test_node.cpp)
ament_target_dependencies(serial_loopback_test_node rclcpp)

add_executable(best_path_planner_node src/best_path_planner.cpp)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(best_path_planner_node PRIVATE -O2)
endif()
ament_target_dependencies(best_path_planner_node
  geometry_msgs
  nav_msgs
  rclcpp
)

install(TARGETS
  hmi_touch_verify_node
  serial_loopback_test_node
  best_path_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
