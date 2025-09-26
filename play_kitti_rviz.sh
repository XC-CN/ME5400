#!/bin/bash

# KITTI数据集RViz播放脚本 (单版本)
# 功能：
#  1. 仅可视化KITTI点云与IMU（不启动FAST-LIO算法）
#  2. 支持可选的RViz配置文件（默认使用 fast_lio 提供的简易配置）
#  3. 支持循环播放 (--loop)
#  4. 支持仅输出命令提示而不自动启动 (--print)
#
# 用法：
#   ./play_kitti_rviz.sh                # 使用默认配置播放一次
#   ./play_kitti_rviz.sh --loop         # 循环播放
#   ./play_kitti_rviz.sh --config your.rviz  # 指定自定义rviz配置
#   ./play_kitti_rviz.sh --norviz       # 不启动rviz（用于仅转换/检查）
#   ./play_kitti_rviz.sh --print        # 仅打印将要执行的命令
#

LOOP=false
RVIZ_CFG="catkin_ws/src/fast_lio/rviz_cfg/kitti_simple.rviz"
START_RVIZ=true
PRINT_ONLY=false

while [[ $# -gt 0 ]]; do
    case $1 in
        --loop)
            LOOP=true; shift ;;
        --config)
            RVIZ_CFG="$2"; shift 2 ;;
        --norviz)
            START_RVIZ=false; shift ;;
        --print)
            PRINT_ONLY=true; shift ;;
        -h|--help)
            grep '^#' "$0" | sed 's/^# //'; exit 0 ;;
        *)
            echo "未知参数: $1"; exit 1 ;;
    esac
done

echo "设置ROS环境..."
source catkin_ws/devel/setup.bash

# 检查是否安装了终端模拟器
if command -v gnome-terminal &> /dev/null; then
    TERMINAL_CMD="gnome-terminal --"
elif command -v xterm &> /dev/null; then
    TERMINAL_CMD="xterm -e"
else
    echo "未找到支持的终端模拟器（gnome-terminal 或 xterm）"
    echo "请手动运行以下命令："
    echo "终端1: roscore"
    echo "终端2: rviz"
    echo "终端3: rosbag play KITTI_Data/kitti_2011_09_26_drive_0019_sync.bag --clock"
    exit 1
fi

CMD_ROSCORE="cd /home/xc/Projects/ME5400 && source catkin_ws/devel/setup.bash && roscore"
CMD_RVIZ="cd /home/xc/Projects/ME5400 && source catkin_ws/devel/setup.bash && rviz -d $RVIZ_CFG"
PLAY_OPT="--clock"
if $LOOP; then PLAY_OPT="$PLAY_OPT --loop"; fi
CMD_BAG="cd /home/xc/Projects/ME5400 && source catkin_ws/devel/setup.bash && rosbag play KITTI_Data/kitti_2011_09_26_drive_0019_sync.bag $PLAY_OPT"

if $PRINT_ONLY; then
    echo "将执行的命令："
    echo "(1) $CMD_ROSCORE"
    if $START_RVIZ; then echo "(2) $CMD_RVIZ"; fi
    echo "(3) $CMD_BAG"
    exit 0
fi

echo "启动ROS核心..."
$TERMINAL_CMD bash -c "$CMD_ROSCORE; exec bash" &

echo "等待2秒让ROS核心启动..."
sleep 2

echo "后台启动KITTI bag数据播放... (参数: $PLAY_OPT)"
$TERMINAL_CMD bash -c "$CMD_BAG; exec bash" &

echo "等待点云第一帧..."
sleep 3

# 自动探测点云frame_id
if command -v rostopic &> /dev/null; then
    DETECTED_FRAME=$(rostopic echo -n1 /kitti/velo/pointcloud 2>/dev/null | grep frame_id | head -1 | awk -F': ' '{print $2}' | tr -d '"[:space:]')
    if [[ -n "$DETECTED_FRAME" ]]; then
        echo "检测到点云 frame_id: $DETECTED_FRAME"
    else
        echo "未能自动检测到 frame_id，可能是bag尚未发布或话题名称错误。"
    fi
else
    echo "未找到 rostopic，跳过 frame_id 自动检测。"
fi

if $START_RVIZ; then
    if [[ -n "$DETECTED_FRAME" ]]; then
        echo "以检测到的 frame_id ($DETECTED_FRAME) 启动RViz..."
        # 若用户提供配置文件，仍使用；否则直接用 -f 设置fixed frame
        if [[ -f "$RVIZ_CFG" ]]; then
            $TERMINAL_CMD bash -c "$CMD_RVIZ -f $DETECTED_FRAME; exec bash" &
        else
            $TERMINAL_CMD bash -c "rviz -f $DETECTED_FRAME; exec bash" &
        fi
    else
        echo "未检测到frame，按原配置启动RViz (如无显示请手动设置 Fixed Frame)。"
        $TERMINAL_CMD bash -c "$CMD_RVIZ; exec bash" &
    fi
else
    echo "已跳过RViz启动 (--norviz)"
fi

echo "等待1秒..."
sleep 1

echo "已启动："
echo "- roscore"
if $START_RVIZ; then echo "- rviz (可视化)"; else echo "- 未启动rviz"; fi
echo "- rosbag 播放 (KITTI)"
echo ""
echo "在RViz中："
echo "1. Fixed Frame 可设置为 'kitti/velo/pointcloud' 或 'map'"
echo "2. PointCloud2 订阅: /kitti/velo/pointcloud"
echo "3. IMU/Odometry (若需要): /kitti/oxts/imu"
echo "4. 循环播放: 使用 --loop 参数"