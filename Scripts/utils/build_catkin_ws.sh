#!/bin/bash
set -e

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
ROOT_DIR=$(cd "$SCRIPT_DIR/../.." && pwd)
CATKIN_WS="$ROOT_DIR/catkin_ws"

source /opt/ros/noetic/setup.bash
cd "$CATKIN_WS"

# Use env -i to completely isolate the build from the current (Conda) environment.
# We only pass through HOME and a minimal PATH, then source ROS and build.
env -i HOME="$HOME" USER="$USER" PATH="/usr/bin:/bin:/opt/ros/noetic/bin" \
    bash -c "source /opt/ros/noetic/setup.bash && cd \"$CATKIN_WS\" && catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCATKIN_ENABLE_TESTING=OFF"
