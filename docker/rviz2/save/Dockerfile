# Use ROS Noetic as the base image
FROM ros:humble

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies
RUN apt-get update && apt-get install -y \
    sudo \
    vim \
    nano \
    net-tools

# Set up rviz2 user
RUN useradd -m rviz2 && echo "rviz2:rviz2" | chpasswd && adduser rviz2 sudo

# Configure sudoers to not require a password for sudo
RUN echo "rviz2 ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Install RViz
RUN apt-get install -y ros-humble-rviz2

# Clean up
RUN rm -rf /var/lib/apt/lists/*  && \
    apt-get clean

# Switch to rviz2 user
USER rviz2

# Add ROS environment variables to .bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /home/rviz2/.bashrc && \
    echo "export ROS_DOMAIN_ID=1" >> /home/rviz2/.bashrc

# Set WORKDIR to src directory and clean package list
WORKDIR /home/rviz2

# Set up entrypoint
ENTRYPOINT ["/bin/bash"]
