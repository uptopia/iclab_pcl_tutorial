#!/usr/bin/env bash

# Get dependent parameters
source "$(dirname "$(readlink -f "${0}")")/get_param.sh"
IMAGE="pcl_tutorial_ros2_ubuntu22"

# docker run --privileged \
docker run --rm \
    --privileged \
    --network=host \
    --ipc=host \
    ${GPU_FLAG} \
    -v /tmp/.Xauthority:/home/"${user}"/.Xauthority \
    -e XAUTHORITY=/home/"${user}"/.Xauthority \
    -e DISPLAY="${DISPLAY}" \
    -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v /etc/timezone:/etc/timezone:ro \
    -v /etc/localtime:/etc/localtime:ro \
    -v /dev:/dev \
    -v ${WS_PATH}/..:/home/${user}/work \
    -it --name "${IMAGE}" "${DOCKER_HUB_USER}"/"${IMAGE}"
    # -it --name "${CONTAINER}" "${DOCKER_HUB_USER}"/"${IMAGE}"

