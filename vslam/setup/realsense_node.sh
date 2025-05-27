#!/bin/bash

# Exit if any command fails
set -e

# Launch RealSense node with IMU and infrared cameras
echo "Launching RealSense camera with IMU and IR streams..."
ros2 launch realsense2_camera rs_launch.py \
  enable_infra1:=true \
  enable_infra2:=true \
  enable_color:=false \
  enable_depth:=false \
  depth_module.emitter_enabled:=0 \
  depth_module.profile:=640x360x90 \
  enable_gyro:=true \
  enable_accel:=true \
  gyro_fps:=200 \
  accel_fps:=200 \
  unite_imu_method:=2
