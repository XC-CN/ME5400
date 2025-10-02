#!/bin/bash
set -e

ROOT_DIR=$(cd "$(dirname "$0")/.." && pwd)
CATKIN_WS="$ROOT_DIR/catkin_ws"
CONFIG="$CATKIN_WS/src/fast_lio/rviz_cfg/kitti_simple.rviz"

if [[ ! -f "$CONFIG" ]]; then
  echo "[错误] 找不到 RViz 配置: $CONFIG" >&2
  exit 1
fi

source Scripts/setup_ros_env.sh
rosrun rviz rviz -d "$CONFIG"
