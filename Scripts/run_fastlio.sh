#!/bin/bash
set -e

ROOT_DIR=$(cd "$(dirname "$0")/.." && pwd)
CATKIN_WS="$ROOT_DIR/catkin_ws"

usage() {
  cat <<'EOF'
用法: run_fastlio.sh [mapping|pose_bridge|both] [参数...]

  mapping       启动 fast_lio 的 mapping_velodyne.launch
  pose_bridge   启动 fastlio_pose_bridge.py
  both          同时启动 mapping 与 pose_bridge（mapping 在后台运行）
  -h, --help    打印本帮助信息

示例:
  ./run_fastlio.sh mapping rviz:=false
  ./run_fastlio.sh pose_bridge
  ./run_fastlio.sh both
EOF
}

if [[ $# -eq 0 ]]; then
  MODE="mapping"
else
  case "$1" in
    -h|--help|help)
      usage
      exit 0
      ;;
    *)
      MODE="$1"
      shift
      ;;
  esac
fi

if [[ ! -d "$CATKIN_WS/devel" ]]; then
  echo "[错误] 请先编译工作空间: $CATKIN_WS" >&2
  exit 1
fi

source "$CATKIN_WS/devel/setup.bash"

case "$MODE" in
  mapping|map)
    if [[ $# -eq 0 ]]; then
      roslaunch fast_lio mapping_velodyne.launch rviz:=false
    else
      roslaunch fast_lio mapping_velodyne.launch "$@"
    fi
    ;;
  pose_bridge|bridge|pose)
    rosrun kitti_tracklets_viz fastlio_pose_bridge.py "$@"
    ;;
  both)
    if [[ $# -gt 0 ]]; then
      echo "[提示] both 模式暂不支持附加参数, 忽略多余参数: $*" >&2
    fi
    roslaunch fast_lio mapping_velodyne.launch rviz:=false &
    MAPPING_PID=$!
    cleanup() {
      if kill -0 "$MAPPING_PID" 2>/dev/null; then
        kill "$MAPPING_PID" 2>/dev/null || true
        wait "$MAPPING_PID" 2>/dev/null || true
      fi
    }
    trap cleanup EXIT INT TERM
    rosrun kitti_tracklets_viz fastlio_pose_bridge.py
    wait "$MAPPING_PID" || true
    ;;
  *)
    echo "[错误] 未知模式: $MODE" >&2
    usage >&2
    exit 1
    ;;
esac
