# Use Issac ROS Dev as the base image
FROM isaac_ros_dev-x86_64:latest

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies
RUN apt-get update && apt-get install -y \
    net-tools \
    ros-humble-isaac-ros-visual-slam \
    ros-humble-isaac-ros-examples \
    ros-humble-isaac-ros-realsense

# Clean up
RUN rm -rf /var/lib/apt/lists/* && \
    apt-get clean

# Change to admin user
USER admin

# Set working directory
WORKDIR /workspaces/isaac_ros-dev

# Build the ROS workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

# Add start up commands to .bashrc
RUN echo "export ROS_DOMAIN_ID=1" >> /home/admin/.bashrc && \
    echo "source /opt/ros/humble/setup.bash" >> /home/admin/.bashrc && \
    echo "source /workspaces/isaac_ros-dev/install/setup.bash" >> /home/admin/.bashrc

# Set up entrypoint
ENTRYPOINT ["/usr/local/bin/scripts/workspace-entrypoint.sh"]
