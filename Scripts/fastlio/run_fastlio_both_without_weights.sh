#!/bin/bash
# 启动 FAST-LIO（both 模式：mapping + pose_bridge，禁用动态权重优化，使用原始方法）
set -e

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
ROOT_DIR=$(cd "$SCRIPT_DIR/../.." && pwd)
CATKIN_WS="$ROOT_DIR/catkin_ws"

if [[ ! -d "$CATKIN_WS/devel" ]]; then
  echo "[错误] 请先编译工作空间: $CATKIN_WS" >&2
  exit 1
fi

source "$CATKIN_WS/devel/setup.bash"

echo "[信息] 启动 FAST-LIO（both 模式，禁用动态权重优化，使用原始方法）"
roslaunch fast_lio mapping_velodyne.launch rviz:=false use_dynamic_weights:=false &
MAPPING_PID=$!

cleanup() {
  if kill -0 "$MAPPING_PID" 2>/dev/null; then
    kill "$MAPPING_PID" 2>/dev/null || true
    wait "$MAPPING_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

rosrun ME5400 fastlio_pose_bridge.py
wait "$MAPPING_PID" || true

