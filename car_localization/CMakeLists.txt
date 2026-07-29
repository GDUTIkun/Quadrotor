cmake_minimum_required(VERSION 3.8)
project(car_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(carto_pose_publisher_node src/carto_pose_publisher_node.cpp)
target_compile_features(carto_pose_publisher_node PRIVATE cxx_std_17)
ament_target_dependencies(carto_pose_publisher_node
  geometry_msgs
  nav_msgs
  rclcpp
  tf2
  tf2_ros
)

install(TARGETS
  carto_pose_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
