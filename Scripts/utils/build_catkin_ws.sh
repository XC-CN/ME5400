#!/bin/bash
set -e

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
ROOT_DIR=$(cd "$SCRIPT_DIR/../.." && pwd)
CATKIN_WS="$ROOT_DIR/catkin_ws"

source /opt/ros/noetic/setup.bash
cd "$CATKIN_WS"
catkin_make
