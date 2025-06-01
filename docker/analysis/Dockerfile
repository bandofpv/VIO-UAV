# Use ROS Noetic as the base image
FROM ubuntu:focal

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies
RUN apt-get update && apt-get install -y \
    git \
    sudo \
    python3 \
    python3-pip \
    net-tools \
    vim \
    nano

# Pre-configure the keyboard layout options to avoid interactive prompts
RUN apt-get update && \
    apt-get install -y debconf-utils && \
    echo 'keyboard-configuration keyboard-configuration/layoutcode select us' | debconf-set-selections && \
    echo 'keyboard-configuration keyboard-configuration/model select Generic 105-key PC (intl.)' | debconf-set-selections

# Setup locale
RUN apt-get update && apt-get install -y locales tzdata && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    ln -fs /usr/share/zoneinfo/America/New_York /etc/localtime && \
    dpkg-reconfigure --frontend noninteractive tzdata
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Set up analysis user
RUN useradd -m analysis && echo "analysis:analysis" | chpasswd && adduser analysis sudo

# Configure sudoers to not require a password for sudo
RUN echo "analysis ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Switch to analysis user
USER analysis

# Install Dependencies for ROS2 Humble
RUN sudo apt-get update && sudo apt-get install -y \
    gnupg \
    wget \
    software-properties-common && \
    sudo add-apt-repository universe

# Setup Source
RUN wget -qO - https://isaac.download.nvidia.com/isaac-ros/repos.key | sudo apt-key add - && \
    grep -qxF "deb https://isaac.download.nvidia.com/isaac-ros/ubuntu/main $(lsb_release -cs) main" /etc/apt/sources.list || \
    echo "deb https://isaac.download.nvidia.com/isaac-ros/ubuntu/main $(lsb_release -cs) main" | sudo tee -a /etc/apt/sources.list && \
    sudo apt-get update && \
    sudo apt-get update && sudo apt-get install curl -y && \
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install core ROS2 Packages
RUN sudo apt-get update && sudo apt-get install -y ros-humble-desktop

# Install ROS2 RealSense Package
RUN sudo apt-get install -y ros-humble-realsense2-camera

# Install RViz2
RUN sudo apt-get install -y ros-humble-rviz2

# Work in home directory
WORKDIR /home/analysis

# Clone VSLAM-UAV repo
RUN git clone https://github.com/bandofpv/VSLAM-UAV.git

# Install Dependencies for allan_ros2 package
RUN sudo apt-get update && sudo apt-get install -y \
    python3-scipy \
    python3-rosdep \
    python3-colcon-common-extensions

# Clean up
RUN sudo rm -rf /var/lib/apt/lists/* && \
    sudo apt-get clean

# Install allan_ros2 and motion_capture_tracking packages
WORKDIR /home/analysis/ros2_ws/src
RUN git clone https://github.com/PX4/px4_msgs.git && \
    git clone https://github.com/CruxDevStuff/allan_ros2.git && \
    git clone --recurse-submodules https://github.com/IMRCLab/motion_capture_tracking.git
WORKDIR /home/analysis/ros2_ws/src/px4_msgs
RUN git reset --hard 87741f4
WORKDIR /home/analysis/ros2_ws/src/motion_capture_tracking
RUN git reset --hard 35adc3e && \
    git submodule update --init --recursive

# Build the ROS2 workspace
WORKDIR /home/analysis/ros2_ws
RUN sudo rosdep init && \
    rosdep update && \
    /bin/bash -c "source /opt/ros/humble/setup.bash && rosdep install --from-paths src -y --ignore-src" && \
    /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

# Copy custom msg files
RUN cp -r /home/analysis/ros2_ws/src/motion_capture_tracking/motion_capture_tracking_interfaces /home/analysis/VSLAM-UAV/validation

# Install Python dependencies for validation
RUN pip3 install pandas rosbags numpy==1.21

# Add start up commands to .bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /home/analysis/.bashrc && \
    echo "source /home/analysis/ros2_ws/install/setup.bash" >> /home/analysis/.bashrc && \
    echo "export ROS_DOMAIN_ID=1" >> /home/analysis/.bashrc

# Start in VSLAM-UAV directory
WORKDIR /home/analysis/VSLAM-UAV

# Set up entrypoint
ENTRYPOINT ["/bin/bash"]
