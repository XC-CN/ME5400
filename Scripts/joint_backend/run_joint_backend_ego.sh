#!/bin/bash
set -e

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
ROOT_DIR=$(cd "$SCRIPT_DIR/../.." && pwd)
CATKIN_WS="$ROOT_DIR/catkin_ws"
DEFAULT_CONFIG="$ROOT_DIR/catkin_ws/src/ME5400/config/joint_backend_ego.yaml"

usage() {
  cat <<USAGE
用法: $0 [--config <path>] [--no-conda]

选项:
  --config <path>  参数配置文件路径（默认: $DEFAULT_CONFIG）
  --no-conda       不自动激活 conda 环境
  -h, --help       显示帮助
USAGE
}

CONFIG_PATH="$DEFAULT_CONFIG"
USE_CONDA=1

while [[ $# -gt 0 ]]; do
  case "$1" in
    --config)
      CONFIG_PATH="$2"
      shift 2
      ;;
    --no-conda)
      USE_CONDA=0
      shift
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

if [[ ! -f "$CONFIG_PATH" ]]; then
  echo "[错误] 配置文件不存在: $CONFIG_PATH" >&2
  exit 1
fi

if [[ ! -d "$CATKIN_WS/devel" ]]; then
  echo "[错误] 请先编译工作空间: $CATKIN_WS" >&2
  exit 1
fi

if [[ $USE_CONDA -eq 1 ]]; then
  # 与现有脚本保持一致：固定 miniconda3 路径
  source ~/miniconda3/etc/profile.d/conda.sh
  conda activate ME5400
fi

source /opt/ros/noetic/setup.bash
source "$CATKIN_WS/devel/setup.bash"

echo "[信息] 加载参数: $CONFIG_PATH"
rosparam load "$CONFIG_PATH" /joint_backend_ego

echo "[信息] 启动 joint_backend_ego_node"
echo "[信息] 输入: /Odometry + /mctrack/tracked_objects"
echo "[信息] 输出: /joint_backend/odom"

rosrun ME5400 joint_backend_ego_node.py __name:=joint_backend_ego
