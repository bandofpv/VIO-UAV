#!/bin/bash
source /opt/ros/noetic/setup.bash
source /opt/ros/humble/setup.bash
source /home/bridge/ros1_bridge/install/setup.bash
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
