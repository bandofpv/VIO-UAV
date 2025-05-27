#!/bin/bash

# Locate the .db3 file
DB3_DIR="/home/analysis/imu_rosbag"
DB3_FILE=$(find "$DB3_DIR" -maxdepth 1 -name "*.db3" | head -n 1)

# Check if a .db3 file was found
if [ -z "$DB3_FILE" ]; then
  echo "No .db3 file found in $DB3_DIR"
  exit 1
fi

echo "Found .db3 file: $DB3_FILE"

# Overwrite the config.yaml file with the found .db3 path
CONFIG_FILE=~/ros2_ws/src/allan_ros2/config/config.yaml

cat << EOF > "$CONFIG_FILE"
allan_node:
  ros__parameters:
     topic: /camera/imu
     bag_path: $DB3_FILE
     msg_type: ros
     publish_rate: 200
     sample_rate: 200
EOF

echo "Configuration file updated successfully at $CONFIG_FILE"

# Change directory to ros2 workspace
cd ~/ros2_ws || { echo "Failed to change directory to ~/ros2_ws"; exit 1; }

# Build the allan_ros2 package
echo "Building allan_ros2 package..."
colcon build --packages-select allan_ros2
if [ $? -ne 0 ]; then
    echo "Build failed."
    exit 1
fi

# Source the workspace
echo "Sourcing the workspace..."
source ~/ros2_ws/install/setup.bash

echo "Build and source complete."
