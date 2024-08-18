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

# Setup locale
RUN apt-get update && apt-get install -y locales tzdata && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    ln -fs /usr/share/zoneinfo/America/New_York /etc/localtime && \
    dpkg-reconfigure --frontend noninteractive tzdata
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Add the ROS 2 apt repository
RUN apt-get install software-properties-common && \
    add-apt-repository universe && \
    apt-get update && apt-get install curl -y && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install development tools and ROS tools
RUN apt-get update && sudo apt-get install -y \
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

# Set up bridge user
RUN useradd -m bridge

# Grant bridge permission to /dev/ttyUSB0
RUN usermod -aG dialout bridge

# Get ROS2 Humble code
RUN mkdir -p /home/bridge/ros2_humble/src
WORKDIR /home/bridge/ros2_humble
RUN vcs import --input https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos src

# Install ROS2 dependencies
RUN apt-get upgrade -y && \
    rosdep init && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"

# Build ROS2 Humble
RUN colcon build --symlink-install

# Install MAVROS
RUN sudo apt-get install -y ros-noetic-mavros ros-noetic-mavros-extras ros-noetic-mavros-msgs && \
    wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh && \
    sudo bash ./install_geographiclib_datasets.sh

# Install pymavlink
RUN pip3 install pymavlink

# Set the working directory for the bridge user
WORKDIR /home/bridge/catkin_ws/src

# Ensure the bridge user has the necessary permissions
RUN mkdir -p /home/bridge/catkin_ws/src && \
    chown -R bridge:bridge /home/bridge/catkin_ws

# Clean up
RUN rm -rf /var/lib/apt/lists/* && \
    apt-get clean

# Switch to bridge user
USER bridge

# Set up ROS1 workspace
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
RUN echo "export ROS_IP=$(hostname -I | awk '{print $1}')" >> /home/bridge/.bashrc && \
    echo "export ROS_MASTER_URI=http://$(hostname -I | awk '{print $1}'):11311" >> /home/bridge/.bashrc

# Insert this line above later --> echo "sudo chown bridge /dev/serial0" >> /home/bridge/.bashrc && \

# Start in MAVROSPY directory
WORKDIR /home/bridge/catkin_ws/src/mavrospy

# Set up entrypoint
ENTRYPOINT ["/bin/bash"]
