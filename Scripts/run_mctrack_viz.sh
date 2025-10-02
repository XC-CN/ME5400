#!/bin/bash
# 一键启动 KITTI rosbag + MCTrack RViz 可视化
set -e

ROOT_DIR=$(cd "$(dirname "$0")/.." && pwd)
BAG_PATH="$ROOT_DIR/KITTI_Data/kitti_2011_09_26_drive_0019_sync.bag"
RESULT_PATH="$ROOT_DIR/MCTrack/results/kitti/20251002_174627/gt/val/data/0000.txt"
CALIB_PATH="$ROOT_DIR/MCTrack/data/kitti/datasets/training/calib/0000.txt"
RVIZ_CFG="$ROOT_DIR/catkin_ws/src/fast_lio/rviz_cfg/kitti_simple.rviz"
FRAME_ID="velo_link"
RATE=10
LOOP=true
NORVIZ=false
PRINT_ONLY=false

usage() {
  cat <<EOF
用法: $0 [选项]
  --bag <path>       指定要播放的 rosbag (默认: $BAG_PATH)
  --result <path>    指定 MCTrack 结果文件 (默认: $RESULT_PATH)
  --calib <path>     指定对应的 calib 文件 (默认: $CALIB_PATH)
  --frame <name>     RViz 中的 Fixed Frame (默认: $FRAME_ID)
  --rate <value>     Marker 发布频率 (默认: $RATE)
  --noloop           不循环播放 rosbag
  --norviz           不自动启动 RViz
  --print            仅打印将要执行的命令
  -h|--help          查看帮助
EOF
}

while [[ $# -gt 0 ]]; do
  case $1 in
    --bag) BAG_PATH="$2"; shift 2;;
    --result) RESULT_PATH="$2"; shift 2;;
    --calib) CALIB_PATH="$2"; shift 2;;
    --frame) FRAME_ID="$2"; shift 2;;
    --rate) RATE="$2"; shift 2;;
    --noloop) LOOP=false; shift;;
    --norviz) NORVIZ=true; shift;;
    --print) PRINT_ONLY=true; shift;;
    -h|--help) usage; exit 0;;
    *) echo "未知参数: $1"; usage; exit 1;;
  esac
done

if [[ ! -f "$BAG_PATH" ]]; then
  echo "[错误] 未找到 rosbag: $BAG_PATH" >&2
  exit 1
fi
if [[ ! -f "$RESULT_PATH" ]]; then
  echo "[错误] 未找到结果文件: $RESULT_PATH" >&2
  exit 1
fi
if [[ ! -f "$CALIB_PATH" ]]; then
  echo "[错误] 未找到 calib 文件: $CALIB_PATH" >&2
  exit 1
fi
if [[ ! -f "$RVIZ_CFG" ]] && [[ "$NORVIZ" == false ]]; then
  echo "[错误] 未找到 RViz 配置: $RVIZ_CFG" >&2
  exit 1
fi

source "$ROOT_DIR/catkin_ws/devel/setup.bash"

if command -v gnome-terminal >/dev/null 2>&1; then
  TERM_CMD="gnome-terminal --"
elif command -v xterm >/dev/null 2>&1; then
  TERM_CMD="xterm -e"
else
  echo "[提示] 未找到 gnome-terminal 或 xterm，请手动执行以下命令："
  echo "  roscore"
  echo "  rosbag play '$BAG_PATH' --clock" $([ "$LOOP" == true ] && echo "--loop")
  echo "  rosrun mctrack_marker_publisher.py (见脚本)"
  echo "  rviz -d '$RVIZ_CFG'"
  exit 0
fi

PLAY_OPTS="--clock"
if [[ "$LOOP" == true ]]; then
  PLAY_OPTS+=" --loop"
fi

ROSCORE_CMD="cd $ROOT_DIR && source catkin_ws/devel/setup.bash && roscore"
BAG_CMD="cd $ROOT_DIR && source catkin_ws/devel/setup.bash && rosbag play '$BAG_PATH' $PLAY_OPTS"
MARKER_CMD="cd $ROOT_DIR && source catkin_ws/devel/setup.bash && python3 Scripts/mctrack_marker_publisher.py --result '$RESULT_PATH' --calib '$CALIB_PATH' --frame '$FRAME_ID' --rate $RATE"
RVIZ_CMD="cd $ROOT_DIR && source catkin_ws/devel/setup.bash && rviz -d '$RVIZ_CFG' -f '$FRAME_ID'"

if [[ "$PRINT_ONLY" == true ]]; then
  echo "命令预览："
  echo "1) $ROSCORE_CMD"
  echo "2) $BAG_CMD"
  echo "3) $MARKER_CMD"
  if [[ "$NORVIZ" == false ]]; then
    echo "4) $RVIZ_CMD"
  fi
  exit 0
fi

if ! pgrep -f "roscore" >/dev/null 2>&1; then
  $TERM_CMD bash -c "$ROSCORE_CMD; exec bash" &
  sleep 2
fi

$TERM_CMD bash -c "$BAG_CMD; exec bash" &
sleep 2
$TERM_CMD bash -c "$MARKER_CMD; exec bash" &
if [[ "$NORVIZ" == false ]]; then
  sleep 2
  $TERM_CMD bash -c "$RVIZ_CMD; exec bash" &
fi

echo "已启动：rosbag 播放 + MCTrack Marker 发布";
[[ "$NORVIZ" == false ]] && echo "RViz 已打开 (配置: $RVIZ_CFG)" || echo "未启用 RViz (--norviz)";
