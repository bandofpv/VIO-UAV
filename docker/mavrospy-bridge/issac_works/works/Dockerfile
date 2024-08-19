# Use ROS Noetic as the base image
FROM ubuntu:focal

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies
RUN apt-get update && apt-get install -y \
    git \
    sudo \
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

# Set up jetson user
RUN useradd -m jetson && echo "jetson:jetson" | chpasswd && adduser jetson sudo

# Configure sudoers to not require a password for sudo
RUN echo "jetson ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Switch to jetson user
USER jetson

# Install Dependencies
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
    sudo apt update && sudo apt install curl -y && \
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install core ROS2 Packages
RUN sudo apt-get update && sudo apt-get install -y ros-humble-desktop

# Install ROS2 RealSense Package
RUN sudo apt install -y ros-humble-realsense2-camera

# Add start up commands to .bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /home/jetson/.bashrc

# Start in home direcotry
WORKDIR /home/jetson

# Clean up
RUN sudo rm -rf /var/lib/apt/lists/* && \
    sudo apt-get clean

# Set up entrypoint
ENTRYPOINT ["/bin/bash"]
