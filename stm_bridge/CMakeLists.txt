cmake_minimum_required(VERSION 3.8)
project(stm_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(stm_bridge_protocol src/protocol.cpp)
target_include_directories(stm_bridge_protocol PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_compile_features(stm_bridge_protocol PUBLIC cxx_std_17)

add_executable(stm_bridge_node src/serial_node.cpp)
target_include_directories(stm_bridge_node PRIVATE include)
target_compile_features(stm_bridge_node PRIVATE cxx_std_17)
ament_target_dependencies(stm_bridge_node
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  tf2_ros
)
target_link_libraries(stm_bridge_node stm_bridge_protocol)

install(TARGETS
  stm_bridge_protocol
  stm_bridge_node
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

install(FILES README.md package.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_protocol test/test_protocol.cpp)
  target_link_libraries(test_protocol stm_bridge_protocol)
endif()

ament_package()

