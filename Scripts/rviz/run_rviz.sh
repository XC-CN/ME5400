#!/bin/bash
set -e

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
ROOT_DIR=$(cd "$SCRIPT_DIR/../.." && pwd)
CONFIG="$SCRIPT_DIR/ME5400.rviz"

if [[ ! -f "$CONFIG" ]]; then
  echo "[错误] 找不到 RViz 配置: $CONFIG" >&2
  exit 1
fi

source "$ROOT_DIR/Scripts/utils/setup_ros_env.sh"
rosrun rviz rviz -d "$CONFIG"
