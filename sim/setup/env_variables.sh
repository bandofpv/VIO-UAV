#!/bin/bash

# Define environment variables to append
CONFIG_LINES=$(cat <<'EOF'

# Isaac Sim root directory
export ISAACSIM_PATH="${HOME}/isaacsim"

# Isaac Sim python executable
alias ISAACSIM_PYTHON="${ISAACSIM_PATH}/python.sh"

# Isaac Sim app
alias ISAACSIM="${ISAACSIM_PATH}/isaac-sim.sh"

# ROS 2 environment variables
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISTRO=humble
export ROS_DOMAIN_ID=1

# Can only be set once per terminal.
# Setting this command multiple times will append the internal library path again potentially leading to conflicts
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$ISAACSIM_PATH/exts/omni.isaac.ros2_bridge/humble/lib
EOF
)

# Append the configuration to ~/.bashrc
echo "$CONFIG_LINES" >> ~/.bashrc
echo "Configuration successfully added to ~/.bashrc"

