#!/bin/bash
export ROS_LOG_DIR=/home/chenyu/dart24_ws/launch_log
source /opt/ros/jazzy/setup.sh
source /home/chenyu/dart24_ws/install/setup.bash
ros2 launch dart_controller dart.launch.py
