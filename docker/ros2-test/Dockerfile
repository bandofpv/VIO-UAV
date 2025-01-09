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
    ros-humble-rviz2

# Set up sim user
RUN useradd -m sim && echo "sim:sim" | chpasswd && adduser sim sudo

# Configure sudoers to not require a password for sudo
RUN echo "sim ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Install Gazebo ROS Packages
# RUN sudo apt-get install -y \
#     ros-noetic-gazebo-ros-pkgs \
#     ros-noetic-gazebo-ros-control

# Install MAVROS
# RUN sudo apt-get install -y \
#     ros-noetic-mavros \
#     ros-noetic-mavros-extras \
#     ros-noetic-mavros-msgs && \
#     wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh && \
#     sudo bash ./install_geographiclib_datasets.sh

# Switch to sim user
USER sim

# Set up PX4 environment
WORKDIR /home/sim
RUN git clone https://github.com/PX4/PX4-Autopilot.git --recursive
RUN bash ./PX4-Autopilot/Tools/setup/ubuntu.sh

# Compile PX4 SITL
WORKDIR /home/sim/PX4-Autopilot
RUN make px4_sitl

# Install Python Dependencies
RUN pip install --user -U empy==3.3.4 pyros-genmsg setuptools packaging

# Set up Micro XRCE-DDS Agent & Client
WORKDIR /home/sim
RUN git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
WORKDIR /home/sim/Micro-XRCE-DDS-Agent/build
RUN cmake .. && \
    make -j$(nproc) && \
    sudo make install && \
    sudo ldconfig /usr/local/lib

# Build ROS2 workspace
WORKDIR /home/sim/ros2_ws/src
RUN git clone https://github.com/PX4/px4_msgs.git && \
    git clone https://github.com/PX4/px4_ros_com.git && \
    git clone https://github.com/ARK-Electronics/ROS2_PX4_Offboard_Example.git
WORKDIR /home/sim/ros2_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

# DELETE LATER
RUN echo "source /opt/ros/humble/setup.bash" >> /home/sim/.bashrc && \
    echo "source /home/sim/ros2_ws/install/local_setup.bash" >> /home/sim/.bashrc && \
    echo "export ROS_DOMAIN_ID=1" >> /home/sim/.bashrc

# Set up ROS workspace
# WORKDIR /home/sim/catkin_ws/src
# RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_init_workspace"
# 
# # Clone MAVROSPY package
# RUN git clone https://github.com/bandofpv/mavrospy.git
# 
# # Install ROS package dependencies
# WORKDIR /home/sim/catkin_ws
# RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && rosdep update" && \
#     /bin/bash -c "source /opt/ros/noetic/setup.bash && rosdep install --from-paths src --ignore-src -r -y"
# 
# # Build the ROS workspace
# RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"
# 
# # Add ROS Environment Setup to .bashrc
# RUN echo "source /opt/ros/noetic/setup.bash" >> /home/sim/.bashrc && \
#     echo "source /home/sim/catkin_ws/devel/setup.bash" >> /home/sim/.bashrc && \
#     echo "source /home/sim/PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash /home/sim/PX4-Autopilot /home/sim/PX4-Autopilot/build/px4_sitl_default" >> /home/sim/.bashrc && \
#     echo "export ROS_PACKAGE_PATH=/home/sim/catkin_ws/src:/opt/ros/noetic/share:/home/sim/PX4-Autopilot:/home/sim/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic" >> /home/sim/.bashrc
# 
# # Set WORKDIR to src directory and clean package list
# WORKDIR /home/sim/catkin_ws/src/mavrospy
# 
# # Clean up
# RUN sudo rm -rf /var/lib/apt/lists/*  && \
#     sudo apt-get clean

# Set up entrypoint
ENTRYPOINT ["/bin/bash"]
