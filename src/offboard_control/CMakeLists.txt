cmake_minimum_required(VERSION 3.8)
project(offboard_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(offboard_node_v2 src/offboard_node_v2.cpp)
add_executable(simple_offboard_node src/simple_offboard_node.cpp)
add_executable(vel_control_offboard_node src/vel_control_offboard_node.cpp)
add_executable(rpi5_servo_pwm_example src/rpi5_servo_pwm_example.cpp)
target_compile_features(rpi5_servo_pwm_example PUBLIC cxx_std_17)

set(OFFBOARD_CONTROL_TARGETS
  offboard_node_v2
  simple_offboard_node
  vel_control_offboard_node
  rpi5_servo_pwm_example
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/offboard_node.cpp")
  add_executable(offboard_node src/offboard_node.cpp)
  list(APPEND OFFBOARD_CONTROL_TARGETS offboard_node)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/continuous_orbit_node.cpp")
  add_executable(continuous_orbit_node continuous_orbit_node.cpp)
  list(APPEND OFFBOARD_CONTROL_TARGETS continuous_orbit_node)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/pd_hover_node.cpp")
  add_executable(pd_hover_node src/pd_hover_node.cpp)
  list(APPEND OFFBOARD_CONTROL_TARGETS pd_hover_node)
endif()

if(TARGET offboard_node)
  ament_target_dependencies(offboard_node
    rclcpp
    geometry_msgs
    mavros_msgs
  )
endif()

ament_target_dependencies(offboard_node_v2
  rclcpp
  geometry_msgs
  mavros_msgs
  std_msgs
)

if(TARGET pd_hover_node)
  ament_target_dependencies(pd_hover_node
    rclcpp
    geometry_msgs
    mavros_msgs
  )
endif()

ament_target_dependencies(simple_offboard_node
  rclcpp
  geometry_msgs
  mavros_msgs
)

ament_target_dependencies(vel_control_offboard_node
  rclcpp
  geometry_msgs
  mavros_msgs
)

if(TARGET continuous_orbit_node)
  ament_target_dependencies(continuous_orbit_node
    rclcpp
    geometry_msgs
    mavros_msgs
  )
endif()

install(TARGETS
  ${OFFBOARD_CONTROL_TARGETS}
  DESTINATION lib/${PROJECT_NAME}
)
set(OFFBOARD_CONTROL_SCRIPTS
  scripts/compare_input_shaping.py
  scripts/plot_pd_velocity.py
  scripts/plot_simple_setpoint_actual.py
)

foreach(script ${OFFBOARD_CONTROL_SCRIPTS})
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${script}")
    install(PROGRAMS
      ${script}
      DESTINATION lib/${PROJECT_NAME}
    )
  endif()
endforeach()
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include")
  # 安装Python模块（关键修正）
  install(DIRECTORY
    include/
    DESTINATION lib/python3.12/site-packages/offboard_control
    PATTERN "*.py"
    PATTERN "__pycache__" EXCLUDE
  )
endif()

set(OFFBOARD_CONTROL_RESOURCE_DIRS)
foreach(resource_dir models cfg)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${resource_dir}")
    list(APPEND OFFBOARD_CONTROL_RESOURCE_DIRS ${resource_dir}/)
  endif()
endforeach()

if(OFFBOARD_CONTROL_RESOURCE_DIRS)
  # 安装资源文件
  install(DIRECTORY
    ${OFFBOARD_CONTROL_RESOURCE_DIRS}
    DESTINATION share/${PROJECT_NAME}
  )
endif()

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
