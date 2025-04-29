# Use ROS Humble as the base image
FROM ros:humble

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies
RUN apt-get update && apt-get install -y \
    sudo \
    wget \
    git \
    nano \
    vim

# Install extra ROS2 packages
RUN apt-get install -y \
    ros-dev-tools \
    ros-humble-topic-tools \
    ros-humble-rviz2

# Install MAVROS dependencies
RUN apt-get install -y python3-vcstool \
    python3-rosinstall-generator \
    python3-osrf-pycommon

# Install MAVROSPY dependencies
RUN apt-get install -y ros-humble-tf-transformations

# Set up sim user
RUN useradd -m sim && echo "sim:sim" | chpasswd && adduser sim sudo

# Configure sudoers to not require a password for sudo
RUN echo "sim ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Switch to sim user
USER sim

# Set up PX4 environment
WORKDIR /home/sim
RUN git clone https://github.com/PX4/PX4-Autopilot.git --recursive
RUN ./PX4-Autopilot/Tools/setup/ubuntu.sh

# Compile PX4 SITL
WORKDIR /home/sim/PX4-Autopilot
RUN make px4_sitl

# Install Python Dependencies
RUN pip install --user -U empy==3.3.4 pyros-genmsg setuptools packaging && \
    pip install --user -U transforms3d --upgrade

# Build ROS2 workspace with MAVROS and MAVROSPY
WORKDIR /home/sim/ros2_ws/src
RUN git clone -b ros2 https://github.com/bandofpv/mavrospy.git
WORKDIR /home/sim/ros2_ws
RUN rosinstall_generator --format repos mavlink | tee /tmp/mavlink.repos && \
    rosinstall_generator --format repos --upstream mavros | tee -a /tmp/mavros.repos && \
    vcs import src < /tmp/mavlink.repos && \
    vcs import src < /tmp/mavros.repos && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y && \
    sudo ./src/mavros/mavros/scripts/install_geographiclib_datasets.sh
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

# Clone VIO-UAV Repository
WORKDIR /home/sim
RUN git clone https://github.com/bandofpv/VIO-UAV.git
WORKDIR /home/sim/VIO-UAV/sim

# Source ROS2 workspace in .bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /home/sim/.bashrc && \
    echo "source /home/sim/ros2_ws/install/setup.bash" >> /home/sim/.bashrc && \
    echo "export ROS_DOMAIN_ID=1" >> /home/sim/.bashrc

# Clean up
RUN sudo rm -rf /var/lib/apt/lists/*  && \
    sudo apt-get clean

# Set up entrypoint
ENTRYPOINT ["/bin/bash"]
