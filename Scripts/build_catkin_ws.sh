#!/bin/bash
set -e

ROOT_DIR=$(cd "$(dirname "$0")/.." && pwd)
CATKIN_WS="$ROOT_DIR/catkin_ws"

source /opt/ros/noetic/setup.bash
cd "$CATKIN_WS"
catkin_make
