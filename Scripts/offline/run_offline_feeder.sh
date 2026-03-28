#!/bin/bash
set -e

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
ROOT_DIR=$(cd "$SCRIPT_DIR/../.." && pwd)
CATKIN_WS="$ROOT_DIR/catkin_ws"
DEFAULT_CONFIG="$ROOT_DIR/catkin_ws/src/ME5400/config/offline_bag_feeder.yaml"

usage() {
  cat <<USAGE
用法: $0 [--config <path>] [--bag <path>]

说明:
  启动半同步离线调度节点（替代 rosbag play）。
  该节点按帧读 bag，并在每个 LiDAR 帧后等待检测/跟踪结果后再推进下一帧。

选项:
  --config <path>  配置文件路径（默认: $DEFAULT_CONFIG）
  --bag <path>     覆盖 bag_path 参数
  -h, --help       显示帮助
USAGE
}

CONFIG_PATH="$DEFAULT_CONFIG"
BAG_PATH=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --config)
      CONFIG_PATH="$2"
      shift 2
      ;;
    --bag)
      BAG_PATH="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[错误] 未知参数: $1" >&2
      usage
      exit 1
      ;;
  esac
done

if [[ ! -d "$CATKIN_WS/devel" ]]; then
  echo "[错误] 请先编译工作空间: $CATKIN_WS" >&2
  exit 1
fi

if [[ ! -f "$CONFIG_PATH" ]]; then
  echo "[错误] 配置文件不存在: $CONFIG_PATH" >&2
  exit 1
fi

source /opt/ros/noetic/setup.bash
source "$CATKIN_WS/devel/setup.bash"

echo "[信息] 设置 use_sim_time=true（离线时钟由 feeder 发布）"
rosparam set use_sim_time true

echo "[信息] 加载配置: $CONFIG_PATH"
rosparam load "$CONFIG_PATH" /offline_bag_feeder

if [[ -n "$BAG_PATH" ]]; then
  if [[ "$BAG_PATH" != /* ]]; then
    if [[ -f "$ROOT_DIR/$BAG_PATH" ]]; then
      BAG_PATH="$ROOT_DIR/$BAG_PATH"
    fi
  fi
  if [[ ! -f "$BAG_PATH" ]]; then
    echo "[错误] 指定的 bag 不存在: $BAG_PATH" >&2
    exit 1
  fi
  echo "[信息] 覆盖 bag_path: $BAG_PATH"
  rosparam set /offline_bag_feeder/bag_path "$BAG_PATH"
else
  CFG_BAG=$(rosparam get /offline_bag_feeder/bag_path 2>/dev/null || true)
  if [[ -n "$CFG_BAG" && "$CFG_BAG" != /* ]]; then
    if [[ -f "$ROOT_DIR/$CFG_BAG" ]]; then
      rosparam set /offline_bag_feeder/bag_path "$ROOT_DIR/$CFG_BAG"
      CFG_BAG="$ROOT_DIR/$CFG_BAG"
    fi
  fi
  if [[ -n "$CFG_BAG" && ! -f "$CFG_BAG" ]]; then
    echo "[错误] 配置中的 bag_path 不存在: $CFG_BAG" >&2
    echo "[提示] 可用 --bag 指定绝对路径，例如: /home/zzy/kitti/tracking/seq_0020_nodet.bag" >&2
    exit 1
  fi
fi

echo "[信息] 启动 offline_bag_feeder"
rosrun ME5400 offline_bag_feeder.py __name:=offline_bag_feeder
