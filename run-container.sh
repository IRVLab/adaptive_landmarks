#!/bin/bash

# Corey Knutson, 5/8/2024

# Run the container with necessary flags, volumes, etc., and interact with the bash shell
docker run --rm --network host -it \
  --user=$(id -u $USER):$(id -g $USER) \
  --volume="/etc/passwd:/etc/passwd:ro" \
  --volume="/etc/shadow:/etc/shadow:ro" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --env="DISPLAY" \
  --volume="$HOME:$HOME" \
  --workdir="$(pwd)" \
  --env ROS_HOSTNAME=localhost \
  --env ROS_MASTER_URI=http://localhost:11311 \
  --privileged \
  rogueraptor7/ros-noetic:latest /bin/bash --rcfile /entrypoint.sh