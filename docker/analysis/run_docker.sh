#!/bin/bash

# Variables
IMAGE_NAME="analysis"
IMAGE_TAG="latest"
DOCKERFILE_PATH="./Dockerfile"
CONTAINER_NAME="analysis-container"
REBUILD=false

# Parse command line arguments
for arg in "$@"; do
    case $arg in
        --rebuild)
            REBUILD=true
            shift
            ;;
    esac
done

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

# If rebuild flag is set, rebuild the image
if [ "$REBUILD" = true ]; then
    echo "Rebuilding Docker image $IMAGE_NAME:$IMAGE_TAG."
    docker build --no-cache --network=host -t "$IMAGE_NAME:$IMAGE_TAG" -f "$DOCKERFILE_PATH" .

    # Check if the rebuild was successful
    if image_exists; then
        echo "Docker image $IMAGE_NAME:$IMAGE_TAG rebuilt successfully."
    else
        echo "Failed to rebuild Docker image $IMAGE_NAME:$IMAGE_TAG."
        exit 1
    fi
fi

# Run docker container
echo "Running $IMAGE_NAME:$IMAGE_TAG."
docker run -it --rm --net=host --name $CONTAINER_NAME -e DISPLAY=$DISPLAY -v ~/VSLAM-UAV/docker/analysis/imu_rosbag:/home/analysis/imu_rosbag -v ~/VSLAM-UAV/docker/analysis/cfg.yaml:/home/analysis/ros2_ws/src/motion_capture_tracking/motion_capture_tracking/config/cfg.yaml -v /tmp/.X11-unix:/tmp/.X11-unix $IMAGE_NAME:$IMAGE_TAG
