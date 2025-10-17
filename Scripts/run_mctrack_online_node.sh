#!/bin/bash
set -e

ROOT_DIR=$(cd "$(dirname "$0")/.." && pwd)
CATKIN_WS="$ROOT_DIR/catkin_ws"
DEFAULT_DATASET="$ROOT_DIR/Data_Tracking/training"
DEFAULT_CONFIG="$ROOT_DIR/MCTrack/config/kitti_fastlio.yaml"
DEFAULT_SEQ=19

usage() {
  cat <<EOF
用法: $0 [选项]
  --dataset <path>   KITTI Tracking 数据集路径 (默认: $DEFAULT_DATASET)
  --config <path>    MCTrack 配置文件 (默认: $DEFAULT_CONFIG)
  --seq <id>         序列号，例如 20 (默认: $DEFAULT_SEQ)
  -h|--help          显示帮助
EOF
}

DATASET_ROOT="$DEFAULT_DATASET"
CONFIG_PATH="$DEFAULT_CONFIG"
SEQ="$DEFAULT_SEQ"

while [[ $# -gt 0 ]]; do
  case $1 in
    --dataset) DATASET_ROOT="$2"; shift 2;;
    --config) CONFIG_PATH="$2"; shift 2;;
    --seq) SEQ="$2"; shift 2;;
    -h|--help) usage; exit 0;;
    *) echo "[错误] 未知参数: $1" >&2; usage; exit 1;;
  esac
done

if [[ ! -d "$CATKIN_WS/devel" ]]; then
  echo "[错误] 请先编译工作空间: $CATKIN_WS" >&2
  exit 1
fi

if [[ ! -d "$DATASET_ROOT" ]]; then
  echo "[错误] 数据集路径不存在: $DATASET_ROOT" >&2
  exit 1
fi

if [[ ! -f "$CONFIG_PATH" ]]; then
  echo "[错误] MCTrack 配置不存在: $CONFIG_PATH" >&2
  exit 1
fi

if ! [[ "$SEQ" =~ ^[0-9]+$ ]]; then
  echo "[错误] 序列号必须是数字: $SEQ" >&2
  exit 1
fi

source "$CATKIN_WS/devel/setup.bash"
rosrun kitti_tracklets_viz mctrack_online_node.py _dataset_root:="$DATASET_ROOT" _config:="$CONFIG_PATH" _seq:=$SEQ
