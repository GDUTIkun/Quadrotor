cmake_minimum_required(VERSION 3.8)
project(track_runner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(track_runner_node src/track_runner_node.cpp)
target_compile_features(track_runner_node PRIVATE cxx_std_17)
ament_target_dependencies(track_runner_node
  geometry_msgs
  nav_msgs
  rclcpp
  std_msgs
)





install(TARGETS
  track_runner_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
