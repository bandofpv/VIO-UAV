# Use mavrospy2-rpi as the base image
FROM mavrospy2-rpi:latest

# Source the ROS and MAVROSPY setup.bash files and launch the outdoor.py file
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /home/rpi/ros2_ws/install/setup.bash && sudo chown rpi /dev/serial0 && ros2 launch mavrospy outdoor.py pattern:=circle"]
