#!/bin/bash

# Get the absolute path to the directory containing this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Build the Docker image from the Dockerfile in the current directory
docker build -t sbmpo:ros2 -f "${SCRIPT_DIR}/Dockerfile.sbmpo-ros2" "${SCRIPT_DIR}"

# Start the Docker container with the current directory mounted, and automatically remove the container when it is stopped or exited
docker run --rm -it \
    --mount type=bind,source="${SCRIPT_DIR}/../",target=/ros2_ws/src/ sbmpo:ros2
