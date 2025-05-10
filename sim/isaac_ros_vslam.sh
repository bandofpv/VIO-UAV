#!/bin/bash

# Exit on error
set -e

# Update package list
echo "Updating package list..."
sudo apt-get update

# Install the ROS2 Humble Isaac ROS Visual SLAM package
echo "Installing isaac-ros-visual-slam..."
sudo apt-get install -y ros-humble-isaac-ros-visual-slam

# Launch the visual SLAM node
echo "Launching Isaac ROS Visual SLAM for Isaac Sim..."
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam_isaac_sim.launch.py
