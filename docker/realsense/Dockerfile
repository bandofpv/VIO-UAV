# Use Ubuntu Focal as the base image
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

# Install Dependencies
RUN apt-get update && apt-get install -y \
    gnupg \
    wget \
    software-properties-common && \
    add-apt-repository universe

# Setup Source
RUN wget -qO - https://isaac.download.nvidia.com/isaac-ros/repos.key | apt-key add - && \
    grep -qxF "deb https://isaac.download.nvidia.com/isaac-ros/ubuntu/main $(lsb_release -cs) main" /etc/apt/sources.list || \
    echo "deb https://isaac.download.nvidia.com/isaac-ros/ubuntu/main $(lsb_release -cs) main" | tee -a /etc/apt/sources.list && \
    apt-get update && \
    apt-get update && apt-get install curl -y && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install core ROS2 Packages
RUN apt-get update && apt-get install -y ros-humble-desktop

# Copy librealsense install scripts
COPY scripts/build-librealsense.sh /opt/realsense/build-librealsense.sh
COPY scripts/install-realsense-dependencies.sh /opt/realsense/install-realsense-dependencies.sh

# Install librealsense
RUN chmod +x /opt/realsense/install-realsense-dependencies.sh && \
    /opt/realsense/install-realsense-dependencies.sh; \
    chmod +x /opt/realsense/build-librealsense.sh && /opt/realsense/build-librealsense.sh -n -v v2.55.1;

# Copy hotplug script for udev rules/hotplug for RealSense
RUN mkdir -p /opt/realsense/
COPY scripts/hotplug-realsense.sh /opt/realsense/hotplug-realsense.sh
COPY udev_rules/99-realsense-libusb-custom.rules /etc/udev/rules.d/99-realsense-libusb.rules

# Install colcon and dependencies
RUN apt-get install python3-colcon-common-extensions ros-humble-diagnostic-updater -y

# Clean up
RUN rm -rf /var/lib/apt/lists/* && \
    apt-get clean

# Set up jetson user
RUN useradd -m jetson && echo "jetson:jetson" | chpasswd && adduser jetson sudo

# Configure sudoers to not require a password for sudo
RUN echo "jetson ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Switch to jetson user
USER jetson

# Clone Realsense ROS
WORKDIR /home/jetson
RUN mkdir -p ros2_ws/src && \
    cd ros2_ws/src && \
    git clone https://github.com/IntelRealSense/realsense-ros.git -b 4.51.1

# Build Realsense ROS
WORKDIR /home/jetson/ros2_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

# Clone VSLAM-UAV repo
WORKDIR /home/jetson
RUN git clone https://github.com/bandofpv/VSLAM-UAV.git

# Start in VSLAM-UAV directory
WORKDIR /home/jetson/VSLAM-UAV

# Add start up commands to .bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /home/jetson/.bashrc && \
    echo "source /home/jetson/ros2_ws/install/setup.bash" >> /home/jetson/.bashrc && \
    echo "export ROS_DOMAIN_ID=1" >> /home/jetson/.bashrc

# Set up entrypoint
ENTRYPOINT ["/bin/bash"]
