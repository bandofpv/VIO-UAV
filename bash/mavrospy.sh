#!/bin/bash
source /opt/ros/noetic/setup.bash
source /home/bridge/catkin_ws/devel/setup.bash
export FCU_URL=/dev/ttyUSB0:921600
# remove export line after rebuilding docker image
