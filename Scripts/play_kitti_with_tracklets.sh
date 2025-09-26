#!/bin/bash
# 同时播放KITTI点云与Tracklet 3D框的脚本
# 功能：
#  1. 播放指定KITTI rosbag
#  2. 启动tracklets_publisher节点发布MarkerArray可视化3D框
#  3. 可选循环播放、可选自定义rviz配置
#  4. 可选仅打印命令 (--print)
#  5. 自动探测pointcloud frame并设置为RViz Fixed Frame
#
# 用法示例：
#   ./play_kitti_with_tracklets.sh \
#       --bag KITTI_Data/kitti_2011_09_26_drive_0019_sync.bag \
#       --tracklet KITTI_Data/2011_09_26/2011_09_26_drive_0019_sync/tracklet_labels.xml
#   ./play_kitti_with_tracklets.sh --loop
#   ./play_kitti_with_tracklets.sh --norviz
#   ./play_kitti_with_tracklets.sh --print
#   ./play_kitti_with_tracklets.sh --frame velo_link
#
# RViz中添加显示：
#  - PointCloud2: /kitti/velo/pointcloud
#  - MarkerArray: /kitti/tracklets_markers
#

set -e
LOOP=true
RVIZ_CFG="catkin_ws/src/fast_lio/rviz_cfg/kitti_simple.rviz"
START_RVIZ=true
PRINT_ONLY=false
BAG="KITTI_Data/kitti_2011_09_26_drive_0019_sync.bag"
TRACKLET_XML="KITTI_Data/2011_09_26/2011_09_26_drive_0019_sync/tracklet_labels.xml"
FRAME_ID="velo_link"
FRAME_RATE=10
TIME_OFFSET=0.0

while [[ $# -gt 0 ]]; do
  case $1 in
    --loop) LOOP=true; shift;;
    --config) RVIZ_CFG="$2"; shift 2;;
    --norviz) START_RVIZ=false; shift;;
    --print) PRINT_ONLY=true; shift;;
    --bag) BAG="$2"; shift 2;;
    --tracklet) TRACKLET_XML="$2"; shift 2;;
    --frame) FRAME_ID="$2"; shift 2;;
    --rate) FRAME_RATE="$2"; shift 2;;
    --offset) TIME_OFFSET="$2"; shift 2;;
    -h|--help) grep '^#' "$0" | sed 's/^# //'; exit 0;;
    *) echo "未知参数: $1"; exit 1;;
  esac
done

if [[ ! -f "$BAG" ]]; then echo "Bag文件不存在: $BAG"; exit 1; fi
if [[ ! -f "$TRACKLET_XML" ]]; then echo "Tracklet XML不存在: $TRACKLET_XML"; exit 1; fi

source catkin_ws/devel/setup.bash

if command -v gnome-terminal &>/dev/null; then
  TERMINAL_CMD="gnome-terminal --"
elif command -v xterm &>/dev/null; then
  TERMINAL_CMD="xterm -e"
else
  echo "未找到终端模拟器 (gnome-terminal/xterm)，请手动运行以下命令："
  echo "1) roscore"
  echo "2) rosbag play $BAG --clock [--loop]"
  echo "3) rosrun kitti_tracklets_viz tracklets_publisher.py _tracklet_file:=$TRACKLET_XML _frame_id:=$FRAME_ID _frame_rate:=$FRAME_RATE _time_offset:=$TIME_OFFSET"
  echo "4) rviz -d $RVIZ_CFG (添加 MarkerArray 话题)"
  exit 1
fi

PLAY_OPT="--clock"
if $LOOP; then PLAY_OPT+=" --loop"; fi
CMD_BAG="cd $(pwd) && source catkin_ws/devel/setup.bash && rosbag play $BAG $PLAY_OPT"
CMD_TRACKLET="cd $(pwd) && source catkin_ws/devel/setup.bash && rosrun kitti_tracklets_viz tracklets_publisher.py _tracklet_file:=$(pwd)/$TRACKLET_XML _frame_id:=$FRAME_ID _frame_rate:=$FRAME_RATE _time_offset:=$TIME_OFFSET"
CMD_RVIZ="cd $(pwd) && source catkin_ws/devel/setup.bash && rviz -d $RVIZ_CFG"

if $PRINT_ONLY; then
  echo "命令预览:"
  echo "1) roscore"
  echo "2) $CMD_BAG"
  echo "3) $CMD_TRACKLET"
  echo "4) $CMD_RVIZ"
  exit 0
fi

# 启动rosbag
$TERMINAL_CMD bash -c "$CMD_BAG; exec bash" &

echo "等待点云发布..."
sleep 2

# 检测 frame
if command -v rostopic &>/dev/null; then
  DETECTED_FRAME=$(rostopic echo -n1 /kitti/velo/pointcloud 2>/dev/null | grep frame_id | head -1 | awk -F': ' '{print $2}' | tr -d '"[:space:]')
  if [[ -n "$DETECTED_FRAME" ]]; then
    echo "检测到 frame_id: $DETECTED_FRAME"
    FRAME_ID="$DETECTED_FRAME"
    # 修改 tracklet 命令以使用检测到的frame
    CMD_TRACKLET="cd $(pwd) && source catkin_ws/devel/setup.bash && rosrun kitti_tracklets_viz tracklets_publisher.py _tracklet_file:=$(pwd)/$TRACKLET_XML _frame_id:=$FRAME_ID _frame_rate:=$FRAME_RATE _time_offset:=$TIME_OFFSET"
  else
    echo "未检测到 frame_id，继续使用: $FRAME_ID"
  fi
fi

# 启动 tracklets 发布
echo "启动 tracklets_publisher (frame: $FRAME_ID)"
$TERMINAL_CMD bash -c "$CMD_TRACKLET; exec bash" &

if $START_RVIZ; then
  echo "启动 RViz"
  $TERMINAL_CMD bash -c "$CMD_RVIZ -f $FRAME_ID; exec bash" &
else
  echo "--norviz 已设置，跳过RViz"
fi

echo "完成。可在RViz中查看 PointCloud2 和 MarkerArray (命名空间: kitti_tracklets)。"
