# Use ROS Humble as the base image
FROM ros:humble

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies
RUN apt-get update && apt-get install -y \
    sudo \
    wget \
    git \
    net-tools \
    python3-pip \
    nano \
    vim

# Install extra ROS2 packages
RUN apt-get install -y \
    ros-dev-tools \
    ros-humble-rviz2 \
    ros-humble-topic-tools

# Install MAVROS dependencies
RUN apt-get install -y python3-vcstool \
    python3-rosinstall-generator \
    python3-osrf-pycommon

# Install MAVROSPY dependencies
RUN apt-get install -y ros-humble-tf-transformations

# Set up jetson user
RUN useradd -m jetson && echo "jetson:jetson" | chpasswd && adduser jetson sudo

# Configure sudoers to not require a password for sudo
RUN echo "jetson ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Grant jetson permission to /dev/ttyUSB0
RUN usermod -aG dialout jetson

# Set the working directory for the jetson user
WORKDIR /home/jetson/ros2_ws/src

# Ensure the jetson user has the necessary permissions
RUN mkdir -p /home/jetson/ros2_ws/src && \
    chown -R jetson:jetson /home/jetson/ros2_ws

# Switch to jetson user
USER jetson

# Install Python Dependencies
RUN pip install --user -U empy==3.3.4 pyros-genmsg setuptools packaging pymavlink && \
    pip install --user -U transforms3d --upgrade

# Build ROS2 workspace with MAVROS, MAVROSPY, and motion_capture_tracking
WORKDIR /home/jetson/ros2_ws/src
RUN git clone -b ros2 https://github.com/bandofpv/mavrospy.git && \
    git clone --recurse-submodules https://github.com/IMRCLab/motion_capture_tracking.git
WORKDIR /home/jetson/ros2_ws
RUN rosinstall_generator --format repos mavlink | tee /tmp/mavlink.repos && \
    rosinstall_generator --format repos --upstream mavros | tee -a /tmp/mavros.repos && \
    vcs import src < /tmp/mavlink.repos && \
    vcs import src < /tmp/mavros.repos && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y && \
    sudo ./src/mavros/mavros/scripts/install_geographiclib_datasets.sh
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

# Source ROS2 workspace in .bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /home/jetson/.bashrc && \
    echo "source /home/jetson/ros2_ws/install/setup.bash" >> /home/jetson/.bashrc && \
    echo "export FCU_URL=/dev/ttyUSB0:921600" >> /home/jetson/.bashrc && \
    echo "export ROS_DOMAIN_ID=1" >> /home/jetson/.bashrc

# Clone VSLAM-UAV repository
WORKDIR /home/jetson
RUN git clone https://github.com/bandofpv/VSLAM-UAV.git

# Clean up
RUN sudo rm -rf /var/lib/apt/lists/*  && \
    sudo apt-get clean

# Set up entrypoint
ENTRYPOINT ["/bin/bash"]
