#!/bin/bash


ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
source $ROOT/scripts/print_color.sh

function usage() 
{
    print_info "Usage: run_dev.sh" {ro2_ws directory path OPTIONAL}
    print_info "Copyright (c) 2021-2022, quandy2020@126.com."
}

# Read and parse config file if exists
#
# CONFIG_IMAGE_KEY (string, can be empty)

if [[ -f "${ROOT}/._ros_common-config" ]]; then
    . "${ROOT}/.isaac_ros_common-config"
fi

ROS_DEV_DIR="$1"
if [[ -z "$ROS_DEV_DIR" ]]; then
    ROS_DEV_DIR="$HOME/workspace/ros2_ws"
    if [[ ! -d "$ROS_DEV_DIR" ]]; then
        ROS_DEV_DIR=$(pwd)
    fi
    print_warning "ros2_ws not specified, assuming $ROS_DEV_DIR"
else
    shift 1
fi

ON_EXIT=()
function cleanup 
{
    for command in "${ON_EXIT[@]}"
    do
        $command
    done
}
trap cleanup EXIT

pushd . >/dev/null
cd $ROOT
ON_EXIT+=("popd")

# Prevent running as root.
if [[ $(id -u) -eq 0 ]]; then
    print_error "This script cannot be executed with root privileges."
    print_error "Please re-run without sudo and follow instructions to configure docker for non-root user if needed."
    exit 1
fi

# Check if user can run docker without root.
RE="\<docker\>"
if [[ ! $(groups $USER) =~ $RE ]]; then
    print_error "User |$USER| is not a member of the 'docker' group and cannot run docker commands without sudo."
    print_error "Run 'sudo usermod -aG docker \$USER && newgrp docker' to add user to 'docker' group, then re-run this script."
    print_error "See: https://docs.docker.com/engine/install/linux-postinstall/"
    exit 1
fi

# Check if able to run docker commands.
if [[ -z "$(docker ps)" ]] ;  then
    print_error "Unable to run docker commands. If you have recently added |$USER| to 'docker' group, you may need to log out and log back in for it to take effect."
    print_error "Otherwise, please check your Docker installation."
    exit 1
fi


PLATFORM="$(uname -m)"

BASE_NAME="ros_dev-$PLATFORM"
CONTAINER_NAME="$BASE_NAME-container"

# Remove any exited containers.
if [ "$(docker ps -a --quiet --filter status=exited --filter name=$CONTAINER_NAME)" ]; then
    docker rm $CONTAINER_NAME > /dev/null
fi

# Re-use existing container.
if [ "$(docker ps -a --quiet --filter status=running --filter name=$CONTAINER_NAME)" ]; then
    print_info "Attaching to running container: $CONTAINER_NAME"
    docker exec -i -t -u admin --workdir /workspace/ros2_ws $CONTAINER_NAME /bin/bash $@
    exit 0
fi

# Build image
IMAGE_KEY=ros2_humble
if [[ ! -z "${CONFIG_IMAGE_KEY}" ]]; then
    IMAGE_KEY=$CONFIG_IMAGE_KEY
fi

BASE_IMAGE_KEY=$PLATFORM.user
if [[ ! -z "${IMAGE_KEY}" ]]; then
    BASE_IMAGE_KEY=$PLATFORM.$IMAGE_KEY

    # If the configured key does not have .user, append it last
    if [[ $IMAGE_KEY != *".user"* ]]; then
        BASE_IMAGE_KEY=$BASE_IMAGE_KEY.user
    fi
fi

print_info "Building $BASE_IMAGE_KEY base as image: $BASE_NAME using key $BASE_IMAGE_KEY"
# $ROOT/scripts/build_base_image.sh $BASE_IMAGE_KEY $BASE_NAME '' '' ''

# if [ $? -ne 0 ]; then
#     print_error "Failed to build base image: $BASE_NAME, aborting."
#     exit 1
# fi

# # Map host's display socket to docker
# DOCKER_ARGS+=("-v /tmp/.X11-unix:/tmp/.X11-unix")
# DOCKER_ARGS+=("-v $HOME/.Xauthority:/home/admin/.Xauthority:rw")
# DOCKER_ARGS+=("-e DISPLAY")
# DOCKER_ARGS+=("-e NVIDIA_VISIBLE_DEVICES=all")
# DOCKER_ARGS+=("-e NVIDIA_DRIVER_CAPABILITIES=all")
# DOCKER_ARGS+=("-e FASTRTPS_DEFAULT_PROFILES_FILE=/usr/local/share/middleware_profiles/rtps_udp_profile.xml")
# DOCKER_ARGS+=("-e ROS_DOMAIN_ID")

# # Optionally load custom docker arguments from file
# DOCKER_ARGS_FILE="$ROOT/.ros_dev-dockerargs"
# if [[ -f "$DOCKER_ARGS_FILE" ]]; then
#     print_info "Using additional Docker run arguments from $DOCKER_ARGS_FILE"
#     readarray -t DOCKER_ARGS_FILE_LINES < $DOCKER_ARGS_FILE
#     for arg in "${DOCKER_ARGS_FILE_LINES[@]}"; do
#         DOCKER_ARGS+=($(eval "echo $arg | envsubst"))
#     done
# fi

# # Run container from image
# print_info "Running $CONTAINER_NAME"
# docker run -it --rm \
#     --privileged \
#     --network host \
#     ${DOCKER_ARGS[@]} \
#     -v $ISAAC_ROS_DEV_DIR:/workspace/ros2_ws \
#     -v /dev/*:/dev/* \
#     -v /etc/localtime:/etc/localtime:ro \
#     --name "$CONTAINER_NAME" \
#     --runtime nvidia \
#     --user="admin" \
#     --entrypoint /usr/local/bin/scripts/workspace-entrypoint.sh \
#     --workdir /workspace/ros2_ws \
#     $@ \
#     $BASE_NAME \
#     /bin/bash
