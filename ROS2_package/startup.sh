#!/bin/bash
cd /home/ubuntu/smart_mbot_ws;sudo su;source /opt/ros/foxy/setup.bash;colcon build --symlink-install;source ./install/setup.bash;ros2 launch smart_mbot_pkg smart_mbot_pkg.launch.py

