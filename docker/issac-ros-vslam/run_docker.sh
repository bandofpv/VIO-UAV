#!/bin/bash

# Variables
IMAGE_NAME="isaac_ros_vslam"
IMAGE_TAG="latest"
DOCKERFILE_PATH="./Dockerfile"
CONTAINER_NAME="isaac_ros_vslam-container"
ISAAC_ROS_DEV_DIR="${ISAAC_ROS_WS}"

DOCKER_ARGS=()

# Map host's display socket to docker
DOCKER_ARGS+=("-v /tmp/.X11-unix:/tmp/.X11-unix")
DOCKER_ARGS+=("-v $HOME/.Xauthority:/home/admin/.Xauthority:rw")
DOCKER_ARGS+=("-e DISPLAY")
DOCKER_ARGS+=("-e NVIDIA_VISIBLE_DEVICES=all")
DOCKER_ARGS+=("-e NVIDIA_DRIVER_CAPABILITIES=all")
DOCKER_ARGS+=("-e FASTRTPS_DEFAULT_PROFILES_FILE=/usr/local/share/middleware_profiles/rtps_udp_profile.xml")
DOCKER_ARGS+=("-e ROS_DOMAIN_ID")
DOCKER_ARGS+=("-e USER")
DOCKER_ARGS+=("-e ISAAC_ROS_WS=/workspaces/isaac_ros-dev")

if [[ $PLATFORM == "aarch64" ]]; then
    DOCKER_ARGS+=("-v /usr/bin/tegrastats:/usr/bin/tegrastats")
    DOCKER_ARGS+=("-v /tmp/:/tmp/")
    DOCKER_ARGS+=("-v /usr/lib/aarch64-linux-gnu/tegra:/usr/lib/aarch64-linux-gnu/tegra")
    DOCKER_ARGS+=("-v /usr/src/jetson_multimedia_api:/usr/src/jetson_multimedia_api")
    DOCKER_ARGS+=("--pid=host")
    DOCKER_ARGS+=("-v /usr/share/vpi3:/usr/share/vpi3")
    DOCKER_ARGS+=("-v /dev/input:/dev/input")

    # If jtop present, give the container access
    if [[ $(getent group jtop) ]]; then
        DOCKER_ARGS+=("-v /run/jtop.sock:/run/jtop.sock:ro")
        JETSON_STATS_GID="$(getent group jtop | cut -d: -f3)"
        DOCKER_ARGS+=("--group-add $JETSON_STATS_GID")
    fi
fi

# Prevent running as root.
if [[ $(id -u) -eq 0 ]]; then
    echo "This script cannot be executed with root privileges."
    echo "Please re-run without sudo and follow instructions to configure docker for non-root user if needed."
    exit 1
fi

# Check if user can run docker without root.
RE="\<docker\>"
if [[ ! $(groups $USER) =~ $RE ]]; then
    echo "User |$USER| is not a member of the 'docker' group and cannot run docker commands without sudo."
    echo "Run 'sudo usermod -aG docker \$USER && newgrp docker' to add user to 'docker' group, then re-run this script."
    echo "See: https://docs.docker.com/engine/install/linux-postinstall/"
    exit 1
fi

# Check if able to run docker commands.
if [[ -z "$(docker ps)" ]];  then
    echo "Unable to run docker commands. If you have recently added |$USER| to 'docker' group, you may need to log out and log back in for it to take effect."
    echo "Otherwise, please check your Docker installation."
    exit 1
fi

# Remove any exited containers.
if [ "$(docker ps -a --quiet --filter status=exited --filter name=$CONTAINER_NAME)" ]; then
    docker rm $CONTAINER_NAME > /dev/null
fi

# Re-use existing container.
if [ "$(docker ps -a --quiet --filter status=running --filter name=$CONTAINER_NAME)" ]; then
    echo "Attaching to running container: $CONTAINER_NAME"
    docker exec -it $CONTAINER_NAME bash
    exit 0
fi

# Function to check if image exists locally
image_exists() {
  docker image inspect "$IMAGE_NAME:$IMAGE_TAG" > /dev/null 2>&1
}

# Check if the image exists
if image_exists; then
  echo "Docker image $IMAGE_NAME:$IMAGE_TAG already exists."
else
  echo "Docker image $IMAGE_NAME:$IMAGE_TAG does not exist. Building it now..."
  docker build --network=host -t "$IMAGE_NAME:$IMAGE_TAG" -f "$DOCKERFILE_PATH" .

  # Check if the build was successful
  if image_exists; then
    echo "Docker image $IMAGE_NAME:$IMAGE_TAG built successfully."
  else
    echo "Failed to build Docker image $IMAGE_NAME:$IMAGE_TAG."
    exit 1
  fi
fi

# Run docker container
echo "Running $IMAGE_NAME:$IMAGE_TAG."
docker run -it --rm \
    --privileged \
    --network host \
    ${DOCKER_ARGS[@]} \
    -v $ISAAC_ROS_DEV_DIR:/workspaces/isaac_ros-dev \
    -v /etc/localtime:/etc/localtime:ro \
    --name "$CONTAINER_NAME" \
    --runtime nvidia \
    $IMAGE_NAME:$IMAGE_TAG \
    /bin/bash
