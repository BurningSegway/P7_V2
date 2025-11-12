#!/bin/bash

# Check if user has Docker permissions
if ! docker info >/dev/null 2>&1; then
    echo "Error: User does not have permission to access Docker daemon. Try adding user to 'docker' group with:"
    echo "  sudo usermod -aG docker $USER"
    echo "Then log out and back in, or run 'newgrp docker'. Alternatively, run this script with sudo."
    exit 1
fi

# Enable X11 forwarding for GUI applications
xhost +local:docker
if [ $? -ne 0 ]; then
    echo "Failed to set up X11 forwarding with xhost. Exiting."
    exit 1
fi

# Function to check if a Docker image exists
check_docker_image() {
    local image_name="$1"
    if ! docker image inspect "$image_name" >/dev/null 2>&1; then
        echo "Docker image $image_name not found."
        return 1
    fi
    echo "Docker image $image_name found."
    return 0
}

# Check and build Summit image if necessary
SUMMIT_IMAGE="summit:latest"
if ! check_docker_image "$SUMMIT_IMAGE"; then
    echo "Building Docker image $SUMMIT_IMAGE..."
    docker build -t "$SUMMIT_IMAGE" -f docker/Dockerfile .
    if [ $? -ne 0 ]; then
        echo "Failed to build Docker image $SUMMIT_IMAGE. Exiting."
        exit 1
    fi
fi

# Run the Docker container with an interactive terminal
docker run -it --rm \
    --privileged \
    --net=host \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v /home/pierre/SummitXL/Summit_xl_base/SafeReinforcementLearning:/home/ros_workspace \
    -v /dev:/dev \
    --name summit \
    summit:latest \
    bash -c "source /opt/ros/melodic/setup.bash && \
             if [ -f /home/ros_workspace/devel/setup.bash ]; then \
                 source /home/ros_workspace/devel/setup.bash; \
             fi && \
             bash"

# Store the exit status of docker run
DOCKER_EXIT_STATUS=$?

# Disable X11 forwarding after container stops
xhost -local:docker

# Check if docker run exited successfully
if [ $DOCKER_EXIT_STATUS -ne 0 ]; then
    echo "Docker run failed with exit status $DOCKER_EXIT_STATUS. Exiting."
    exit $DOCKER_EXIT_STATUS
fi
