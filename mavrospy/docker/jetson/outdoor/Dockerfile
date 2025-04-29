# Use mavrospy2-jetson as the base image
FROM mavrospy2-jetson:latest

# Source the ROS and MAVROSPY setup.bash files and launch the outdoor.py file
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /home/jetson/ros2_ws/install/setup.bash && ros2 launch mavrospy outdoor.py fcu_url:=/dev/ttyUSB0:921600 pattern:=circle"]
