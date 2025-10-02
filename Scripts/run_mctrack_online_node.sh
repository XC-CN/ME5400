#!/bin/bash
set -e

ROOT_DIR=$(cd "$(dirname "$0")/.." && pwd)
CATKIN_WS="$ROOT_DIR/catkin_ws"

if [[ ! -d "$CATKIN_WS/devel" ]]; then
  echo "[错误] 请先编译工作空间: $CATKIN_WS" >&2
  exit 1
fi

source "$CATKIN_WS/devel/setup.bash"
rosrun kitti_tracklets_viz mctrack_online_node.py
