cmake_minimum_required(VERSION 3.8)
project(gpio_keys)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/gpio_keys_node.py
  scripts/register_gpio_keys.sh
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
