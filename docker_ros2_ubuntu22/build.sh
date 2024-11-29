#!/usr/bin/env bash

# Enable docker build kit function
export DOCKER_BUILDKIT=1

# Get dependent parameters
source "$(dirname "$(readlink -f "${0}")")/get_param.sh"
IMAGE="pcl_tutorial_ros2_ubuntu22"

# Build docker images
docker build -t "${DOCKER_HUB_USER}"/"${IMAGE}" \
    --build-arg USER="${user}" \
    --build-arg UID="${uid}" \
    --build-arg GROUP="${group}" \
    --build-arg GID="${gid}" \
    --build-arg HARDWARE="${hardware}" \
    --build-arg ENTRYPOINT_FILE="${ENTRYPOINT_FILE}" \
    -f "${FILE_DIR}"/"${DOCKERFILE_NAME}" "${FILE_DIR}"

#     --progress=plain \
#     --no-cache \

