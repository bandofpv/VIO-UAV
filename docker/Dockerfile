# Use ROS Noetic as the base image
FROM ros:noetic

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies
RUN apt-get update && apt-get install -y \
    git \
    sudo \
    net-tools \
    python3-pip \
    python3-colcon-common-extensions \
    vim \
    nano

# Setup locale
RUN apt-get update && apt-get install -y locales tzdata && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    ln -fs /usr/share/zoneinfo/America/New_York /etc/localtime && \
    dpkg-reconfigure --frontend noninteractive tzdata
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Install Dependencies
RUN apt-get update && sudo apt-get install -y \
    gnupg \
    wget \
    software-properties-common && \
    sudo add-apt-repository universe

# Setup Source
RUN wget -qO - https://isaac.download.nvidia.com/isaac-ros/repos.key | sudo apt-key add - && \
    grep -qxF "deb https://isaac.download.nvidia.com/isaac-ros/ubuntu/main $(lsb_release -cs) main" /etc/apt/sources.list || \
    echo "deb https://isaac.download.nvidia.com/isaac-ros/ubuntu/main $(lsb_release -cs) main" | sudo tee -a /etc/apt/sources.list && \
    apt-get update && sudo apt-get install curl -y && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install core ROS2 Packages
RUN apt-get update && apt-get install -y ros-humble-desktop

# Install MAVROS
RUN apt-get install -y ros-noetic-mavros ros-noetic-mavros-extras ros-noetic-mavros-msgs && \
    wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh && \
    bash ./install_geographiclib_datasets.sh

# Install pymavlink
RUN pip3 install pymavlink

# Set up bridge user
RUN useradd -m bridge && echo "bridge:bridge" | chpasswd && adduser bridge sudo

# Configure sudoers to not require a password for sudo
RUN echo "bridge ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Set the working directory for the bridge user
WORKDIR /home/bridge/catkin_ws/src

# Ensure the bridge user has the necessary permissions
RUN mkdir -p /home/bridge/catkin_ws/src && \
    chown -R bridge:bridge /home/bridge/catkin_ws

# Switch to bridge user
USER bridge

# Set up ROS1 workspace
WORKDIR /home/bridge/catkin_ws/src
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_init_workspace"

# Clone QTM ROS package
RUN git clone https://github.com/KTH-SML/motion_capture_system.git

# Install ROS1 package dependencies
WORKDIR /home/bridge/catkin_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && rosdep update" && \
    /bin/bash -c "source /opt/ros/noetic/setup.bash && rosdep install --from-paths src --ignore-src -r -y"

# Build the ROS1 workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Build ros1_bridge
RUN mkdir -p /home/bridge/ros1_bridge/src
WORKDIR /home/bridge/ros1_bridge
RUN git clone https://github.com/ros2/ros1_bridge src/ && \
    /bin/bash -c "source /opt/ros/noetic/setup.bash && \
                  source /home/bridge/catkin_ws/devel/setup.sh && \
                  source /opt/ros/humble/setup.bash && \
                  colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure"

# Add start up commands to .bashrc
RUN echo "export ROS_IP=$(hostname -I | awk '{print $1}')" >> /home/bridge/.bashrc && \
    echo "export ROS_DOMAIN_ID=1" >> /home/bridge/.bashrc

# Clone VIO-UAV directory
WORKDIR /home/bridge
RUN git clone https://github.com/bandofpv/VIO-UAV.git

# Clean up
RUN sudo rm -rf /var/lib/apt/lists/* && \
    sudo apt-get clean

# Start in VIO-UAV directory
WORKDIR /home/bridge/VIO-UAV

# Set up entrypoint
ENTRYPOINT ["/bin/bash"]
