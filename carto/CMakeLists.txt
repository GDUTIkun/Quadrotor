cmake_minimum_required(VERSION 3.8)
project(carto)

find_package(ament_cmake REQUIRED)

install(FILES
  backpack_2d_localization.lua
  my_laser_with_imu.lua
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  my_backpack_2d_localization.launch.py
  my_laser_with_imu.launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
