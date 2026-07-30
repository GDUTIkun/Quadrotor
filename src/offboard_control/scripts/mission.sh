#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash
source /home/pi5/flight_ws/install/setup.bash

export PATH="$PATH:/home/pi5/.local/bin"

export ROS_DOMAIN_ID=0
export ROS_MASTER_URI=http://192.168.50.1:11311

exec ros2 launch track2vision tracked2vision.launch.py
