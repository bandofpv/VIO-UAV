#!/bin/bash

# Update package list
echo "Updating package list..."
sudo apt-get update

# Install ROS Humble Isaac ROS Visual SLAM
echo "Installing isaac-ros-visual-slam..."
sudo apt-get install -y ros-humble-isaac-ros-visual-slam

# Install example and RealSense Isaac ROS packages
echo "Installing isaac-ros-examples and isaac-ros-realsense..."
sudo apt-get install -y ros-humble-isaac-ros-examples ros-humble-isaac-ros-realsense

# Set ROS domain ID (session scope only)
echo "Setting ROS_DOMAIN_ID to 1..."
export ROS_DOMAIN_ID=1

# Launch the visual SLAM package with RealSense
echo "Launching Isaac ROS Visual SLAM with RealSense..."
ros2 launch isaac_ros_vslam_realsense.py
