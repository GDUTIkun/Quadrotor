cmake_minimum_required(VERSION 3.8)
project(quadrotor_ground_station)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(hmi_touch_verify_node src/hmi_touch_verify_node.cpp)
ament_target_dependencies(hmi_touch_verify_node rclcpp)

install(TARGETS
  hmi_touch_verify_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
