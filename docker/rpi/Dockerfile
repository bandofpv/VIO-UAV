# Use ROS Noetic as the base image
FROM ros:noetic

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies
RUN apt-get update && apt-get install -y \
    git \
    sudo \
    wget \
    net-tools \
    python3-pip \
    vim \
    nano

# Set up rpi user
RUN useradd -m rpi && echo "rpi:rpi" | chpasswd && adduser rpi sudo

# Configure sudoers to not require a password for sudo
RUN echo "rpi ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Install MAVROS
RUN sudo apt-get install -y ros-noetic-mavros ros-noetic-mavros-extras ros-noetic-mavros-msgs && \
    wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh && \
    sudo bash ./install_geographiclib_datasets.sh

# Switch to rpi user
USER rpi

# Install pymavlink
RUN pip3 install pymavlink

# Set up ROS workspace
WORKDIR /home/rpi/catkin_ws/src
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_init_workspace"

# Clone MAVROSPY repo
RUN git clone https://github.com/bandofpv/mavrospy.git

# Install ROS package dependencies
WORKDIR /home/rpi/catkin_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && rosdep update" && \
    /bin/bash -c "source /opt/ros/noetic/setup.bash && rosdep install --from-paths src --ignore-src -r -y"

# Build the ROS workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Add start up commands to .bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> /home/rpi/.bashrc && \
    echo "source /home/rpi/catkin_ws/devel/setup.bash" >> /home/rpi/.bashrc && \
    echo "sudo chown rpi /dev/serial0" >> /home/rpi/.bashrc && \
    echo "export ROS_IP=$(hostname -I | awk '{print $1}')" >> /home/rpi/.bashrc && \
    echo "export ROS_MASTER_URI=http://$(hostname -I | awk '{print $1}'):11311" >> /home/rpi/.bashrc

# Start in MAVROSPY direcotry
WORKDIR /home/rpi/catkin_ws/src/mavrospy

# Clean up
RUN sudo rm -rf /var/lib/apt/lists/* && \
    sudo apt-get clean

# Set up entrypoint
ENTRYPOINT ["/bin/bash"]
