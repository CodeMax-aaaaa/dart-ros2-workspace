#!/bin/zsh
export ROS_LOG_DIR=/home/chenyu/dart24_ws/launch_log
source /home/chenyu/.zshrc
source /home/chenyu/dart24_ws/install/setup.zsh
# 打印环境变量
printenv | grep ROS
ros2 launch dart_controller dart.launch.py
