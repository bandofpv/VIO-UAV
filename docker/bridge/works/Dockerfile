# Use ROS Noetic as the base image
FROM ros:noetic

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive

# Add universe repository
RUN apt-get update && \
    apt-get install -y software-properties-common && \
    add-apt-repository universe

# Install dependencies
RUN apt-get update && apt-get install -y \
    git \
    sudo \
    wget \
    net-tools \
    python3-pip \
    vim \
    nano \
    build-essential \
    cmake \
    python3-pytest-cov

# Install colcon from PyPI, rather than apt packages
RUN python3 -m pip install -U colcon-common-extensions vcstool

# Set up bridge user
RUN useradd -m bridge && echo "bridge:bridge" | chpasswd && adduser bridge sudo

# Configure sudoers to not require a password for sudo
RUN echo "bridge ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Install MAVROS
RUN sudo apt-get install -y ros-noetic-mavros ros-noetic-mavros-extras ros-noetic-mavros-msgs && \
    wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh && \
    sudo bash ./install_geographiclib_datasets.sh

# Switch to bridge user
USER bridge

# Set locale
RUN sudo apt-get update && sudo apt-get install locales && \
    sudo locale-gen en_US en_US.UTF-8 && \
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    export LANG=en_US.UTF-8

# Add the ROS 2 apt repository
RUN sudo apt-get install software-properties-common && \
    sudo add-apt-repository universe && \
    sudo apt-get update && sudo apt install curl -y && \
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install development tools and ROS tools
RUN sudo apt update && sudo apt install -y \
    python3-flake8-docstrings \
    ros-dev-tools && \
    python3 -m pip install -U \
    flake8-blind-except \
    flake8-builtins \
    flake8-class-newline \
    flake8-comprehensions \
    flake8-deprecated \
    flake8-import-order \
    flake8-quotes \
    "pytest>=5.3" \
    pytest-repeat \
    pytest-rerunfailures

# Get ROS2 Humble code
RUN mkdir -p /home/bridge/ros2_humble/src
WORKDIR /home/bridge/ros2_humble
RUN vcs import --input https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos src

# Install ROS2 dependencies
RUN sudo apt-get upgrade -y && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"

# Build ROS2 Humble
RUN colcon build --symlink-install

# Install pymavlink
RUN pip3 install pymavlink

# Set up ROS workspace
WORKDIR /home/bridge/catkin_ws/src
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_init_workspace"

# Clone MAVROSPY repo
RUN git clone https://github.com/bandofpv/mavrospy.git

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
                  source /home/bridge/ros2_humble/install/setup.bash && \
                  colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure"

# Add start up commands to .bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> /home/bridge/.bashrc && \
    echo "source /home/bridge/catkin_ws/devel/setup.bash" >> /home/bridge/.bashrc && \
    echo "source /home/bridge/ros2_humble/install/setup.bash" >> /home/bridge/.bashrc && \
    echo "source /home/bridge/ros1_bridge/install/setup.bash" >> /home/bridge/.bashrc && \
    echo "export ROS_IP=$(hostname -I | awk '{print $1}')" >> /home/bridge/.bashrc && \
    echo "export ROS_MASTER_URI=http://$(hostname -I | awk '{print $1}'):11311" >> /home/bridge/.bashrc

# Insert this line above later --> echo "sudo chown bridge /dev/serial0" >> /home/bridge/.bashrc && \

# Start in MAVROSPY direcotry
WORKDIR /home/bridge/catkin_ws/src/mavrospy

# Clean up
RUN sudo rm -rf /var/lib/apt/lists/* && \
    sudo apt-get clean

# Set up entrypoint
ENTRYPOINT ["/bin/bash"]
