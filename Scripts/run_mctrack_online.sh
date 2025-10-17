#!/bin/bash
set -e

ROOT_DIR=$(cd "$(dirname "$0")/.." && pwd)
CATKIN_WS="$ROOT_DIR/catkin_ws"
DEFAULT_BAG="$ROOT_DIR/tracking/rosbags/seq_0019_with_det.bag"
DEFAULT_CONFIG="$ROOT_DIR/MCTrack/config/kitti_fastlio.yaml"
DEFAULT_SEQ=19
DEFAULT_DATASET="$ROOT_DIR/tracking/training"
RVIZ_CFG="$ROOT_DIR/Scripts/rviz_cfg/ME5400.rviz"
FRAME_ID="velo_link"

usage() {
  cat <<EOF
用法: $0 [选项]
  --bag <path>             KITTI 点云+IMU rosbag (默认: $DEFAULT_BAG)
  --dataset <path>         KITTI Tracking 根目录 (默认: $DEFAULT_DATASET)
  --seq <id>               序列号 (默认: $DEFAULT_SEQ)
  --config <path>          MCTrack 配置文件 (默认: $DEFAULT_CONFIG)
  --norviz                 不启动 RViz
  --noloop                 rosbag 不循环
  --print                  仅打印命令
  -h|--help                显示帮助

提示：请先单独启动 PointPillars 检测节点，确保其发布 /detection/kitti_tracking。
EOF
}

BAG_PATH="$DEFAULT_BAG"
DATASET_ROOT="$DEFAULT_DATASET"
SEQ="$DEFAULT_SEQ"
CONFIG_PATH="$DEFAULT_CONFIG"
NORVIZ=false
LOOP=true
PRINT_ONLY=false

while [[ $# -gt 0 ]]; do
  case $1 in
    --bag) BAG_PATH="$2"; shift 2;;
    --dataset) DATASET_ROOT="$2"; shift 2;;
    --seq) SEQ="$2"; shift 2;;
    --config) CONFIG_PATH="$2"; shift 2;;
    --norviz) NORVIZ=true; shift;;
    --noloop) LOOP=false; shift;;
    --print) PRINT_ONLY=true; shift;;
    -h|--help) usage; exit 0;;
    *) echo "未知参数: $1" >&2; usage; exit 1;;
  esac
done

if [[ ! -f "$BAG_PATH" ]]; then
  echo "[错误] bag 文件不存在: $BAG_PATH" >&2
  exit 1
fi
if [[ ! -d "$DATASET_ROOT" ]]; then
  echo "[错误] 数据集路径不存在: $DATASET_ROOT" >&2
  exit 1
fi
if [[ ! -f "$CONFIG_PATH" ]]; then
  echo "[错误] 配置文件不存在: $CONFIG_PATH" >&2
  exit 1
fi
if [[ "$NORVIZ" == false && ! -f "$RVIZ_CFG" ]]; then
  echo "[错误] RViz 配置不存在: $RVIZ_CFG" >&2
  exit 1
fi

source "$CATKIN_WS/devel/setup.bash"

PLAY_OPTS="--clock"
if $LOOP; then
  PLAY_OPTS+=" --loop"
fi

ROSCORE_CMD="roscore"
BAG_CMD="rosbag play '$BAG_PATH' $PLAY_OPTS"
POSE_CMD="rosrun kitti_tracklets_viz fastlio_pose_bridge.py"
TRACK_CMD="rosrun kitti_tracklets_viz mctrack_online_node.py _dataset_root:=$DATASET_ROOT _seq:=$SEQ _config:=$CONFIG_PATH"
RVIZ_CMD="rviz -d '$RVIZ_CFG' -f '$FRAME_ID'"

if $PRINT_ONLY; then
  cat <<EOF
计划执行的命令：
1) $ROSCORE_CMD
2) $BAG_CMD
3) $POSE_CMD
4) $TRACK_CMD
EOF
  if [[ "$NORVIZ" == false ]]; then
    echo "5) $RVIZ_CMD"
  fi
  exit 0
fi

launch_terminal() {
  if command -v gnome-terminal >/dev/null 2>&1; then
    gnome-terminal -- bash -lc "$1; exec bash"
  elif command -v xterm >/dev/null 2>&1; then
    xterm -e bash -lc "$1; exec bash"
  else
    echo "请手动运行: $1" >&2
  fi
}

if ! pgrep -f "roscore" >/dev/null 2>&1; then
  launch_terminal "$ROSCORE_CMD"
  sleep 2
fi

launch_terminal "$BAG_CMD"
sleep 2
launch_terminal "$POSE_CMD"
sleep 2
launch_terminal "$TRACK_CMD"

if [[ "$NORVIZ" == false ]]; then
  sleep 2
  launch_terminal "$RVIZ_CMD"
fi

echo "所有节点已启动。请确保 PointPillars 检测节点已发布 /detection/kitti_tracking。里程计: /mctrack/lidar_pose, 结果: /mctrack/markers"
