#!/bin/bash
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
ROOT_DIR=$(cd "$SCRIPT_DIR/.." && pwd)
CATKIN_WS="$ROOT_DIR/catkin_ws"

source /opt/ros/noetic/setup.bash
if [[ -d "$CATKIN_WS/devel" ]]; then
  source "$CATKIN_WS/devel/setup.bash"
else
  echo "[警告] 未找到 $CATKIN_WS/devel，已仅加载系统 ROS 环境" >&2
fi
