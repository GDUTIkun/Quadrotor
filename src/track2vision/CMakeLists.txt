cmake_minimum_required(VERSION 3.8)
project(track2vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(pose_forwarder src/tracked2vision.cpp)
ament_target_dependencies(
  pose_forwarder
  rclcpp
  geometry_msgs
  std_msgs
)

add_executable(cartographer_laser_transfer src/laser_transfer.cpp)
ament_target_dependencies(
  cartographer_laser_transfer
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

add_executable(imu_covariance_filter src/imu_covariance_filter.cpp)
ament_target_dependencies(
  imu_covariance_filter
  rclcpp
  sensor_msgs
)

add_executable(tf_broadcaster_node src/tf_broadcaster.cpp)
ament_target_dependencies(
  tf_broadcaster_node
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
)

install(
  TARGETS
  pose_forwarder
  cartographer_laser_transfer
  imu_covariance_filter
  tf_broadcaster_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES
  src/tracked2vision.launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  PROGRAMS
  scripts/tf_jump_recorder.py
  scripts/record_ev_local_yaw.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
