#!/bin/bash
set -e

ROOT_DIR=$(cd "$(dirname "$0")/.." && pwd)
CONFIG="$ROOT_DIR/Scripts/rviz_cfg/ME5400.rviz"

if [[ ! -f "$CONFIG" ]]; then
  echo "[错误] 找不到 RViz 配置: $CONFIG" >&2
  exit 1
fi

source "$ROOT_DIR/Scripts/setup_ros_env.sh"
rosrun rviz rviz -d "$CONFIG"
