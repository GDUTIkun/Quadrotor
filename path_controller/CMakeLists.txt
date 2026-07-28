cmake_minimum_required(VERSION 3.8)
project(path_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(path_controller_core src/control.cpp)
target_include_directories(path_controller_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_compile_features(path_controller_core PUBLIC cxx_std_17)

add_executable(path_controller_node src/path_controller_node.cpp)
target_include_directories(path_controller_node PRIVATE include)
target_compile_features(path_controller_node PRIVATE cxx_std_17)
ament_target_dependencies(path_controller_node
  geometry_msgs
  nav_msgs
  rclcpp
  std_msgs
  tf2
  tf2_ros
)
target_link_libraries(path_controller_node path_controller_core)

install(TARGETS
  path_controller_core
  path_controller_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_control test/test_control.cpp)
  target_link_libraries(test_control path_controller_core)
endif()

ament_package()

