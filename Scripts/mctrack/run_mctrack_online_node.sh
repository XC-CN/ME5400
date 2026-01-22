#!/bin/bash
set -e

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
ROOT_DIR=$(cd "$SCRIPT_DIR/../.." && pwd)
CATKIN_WS="$ROOT_DIR/catkin_ws"
DEFAULT_CONFIG="$ROOT_DIR/MCTrack/config/kitti_fastlio.yaml"

# 激活conda环境 (与 PointPillars 保持一致)
source ~/miniconda3/etc/profile.d/conda.sh
conda activate ME5400

usage() {
  cat <<EOF
用法: $0 [选项]
  --config <path>    MCTrack 配置文件 (默认: $DEFAULT_CONFIG)
  -h|--help          显示帮助
  
注意: 
- PointPillars直接输出LiDAR坐标系检测，MCTrack无需相机标定文件
- MCTrack是纯跟踪算法，只需要检测框和位姿信息
EOF
}

CONFIG_PATH="$DEFAULT_CONFIG"

while [[ $# -gt 0 ]]; do
  case $1 in
    --config) CONFIG_PATH="$2"; shift 2;;
    -h|--help) usage; exit 0;;
    *) echo "[错误] 未知参数: $1" >&2; usage; exit 1;;
  esac
done

if [[ ! -d "$CATKIN_WS/devel" ]]; then
  echo "[错误] 请先编译工作空间: $CATKIN_WS" >&2
  exit 1
fi

if [[ ! -f "$CONFIG_PATH" ]]; then
  echo "[错误] MCTrack 配置不存在: $CONFIG_PATH" >&2
  exit 1
fi

source "$CATKIN_WS/devel/setup.bash"

echo "[信息] 启动MCTrack在线跟踪节点"
echo "[信息] 订阅话题: /detection/lidar_detections (Detection3DArray, 带时间戳)"
echo "[信息]           /mctrack/lidar_pose (FAST-LIO 位姿)"

rosrun ME5400 mctrack_online_node.py \
  _config:="$CONFIG_PATH"
