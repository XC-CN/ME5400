#!/bin/bash
# -----------------------------------------------------------------------------
# 功能: 仅可视化 KITTI tracklet 3D 边界框 (含轨迹/文字) 并自动加载预设 RViz 配置。
# 目标: 修复之前脚本的若干问题，并支持自动外参与优雅清理。
# -----------------------------------------------------------------------------
# 修复点:
#  1) 之前使用了错误的节点名 tracklet_publisher -> 正确应为 tracklets_publisher.py
#  2) 未以参数形式传递 ~tracklet_file (节点只用参数，不解析位置参数)
#  3) 未启动 roscore 却在结尾 kill $ROSCORE_PID
#  4) 没有加载专用 RViz 配置文件
#  5) 未提供 camera->velo 外参 (导致框位置与点云坐标系不一致)
#  6) 缺少清理 (Ctrl+C) 逻辑与健壮性
# -----------------------------------------------------------------------------
# 用法示例:
#   ./Scripts/tracklets.sh                               # 使用默认 tracklet + 默认 rviz 配置
#   ./Scripts/tracklets.sh -t path/to/tracklet_labels.xml
#   ./Scripts/tracklets.sh -c catkin_ws/src/kitti_tracklets_viz/rviz/tracklet_debug.rviz
#   ./Scripts/tracklets.sh --types Car,Pedestrian --no-traj
#   ./Scripts/tracklets.sh --no-rviz
# -----------------------------------------------------------------------------
# 主要参数:
#   -t|--tracklet <file>         指定 tracklet_labels.xml
#   -c|--config  <rviz.rviz>     指定 RViz 配置
#   -r|--rate <Hz>               发布帧率 (默认10)
#      --frame <frame_id>        目标显示 frame (默认 velo_link)
#      --types Car,Pedestrian    仅显示这些类别 (逗号分隔)
#      --no-text                 关闭文本标注
#      --no-traj                 关闭轨迹折线
#      --traj-len N              轨迹历史长度 (默认300)
#      --traj-width W            轨迹线宽 (m)
#      --coord camera|lidar      Tracklet坐标系 (默认camera)
#      --debug                   更多日志
#      --no-rviz                 不启动 RViz
#      --sim-time                设置 use_sim_time=true (若后续要与 rosbag 同步)
#      --print-only              仅打印最终命令不执行
# -----------------------------------------------------------------------------
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
WS_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
cd "$WS_ROOT"

# 默认参数
TRACKLET_XML="KITTI_Data/2011_09_26/2011_09_26_drive_0019_sync/tracklet_labels.xml"
RVIZ_CFG="catkin_ws/src/kitti_tracklets_viz/rviz/tracklet_debug.rviz"
FRAME_ID="velo_link"
FRAME_RATE=10
INCLUDE_TYPES=""
ENABLE_TEXT=1
ENABLE_TRAJ=1
TRAJ_HISTORY_LEN=300
TRAJ_WIDTH=0.08
TRACKLET_COORD="camera"   # camera 坐标(需要外参) 或 lidar
DEBUG_MODE=0
START_RVIZ=1
SET_SIM_TIME=0
PRINT_ONLY=0

# KITTI 相机->Velodyne 外参 (p_velo = R * p_cam + t)
# 由官方标定文件 Tr_velo_to_cam 反求得 (序列常见值)，可根据自己数据集替换。
VELO_FROM_CAM_R="0.007533745,0.01480249,0.9998621,-0.9999714,0.0007280733,0.00752379,-0.0006166020,-0.9998902,0.01480755"
VELO_FROM_CAM_T="0.272903,-0.00197,-0.072288"

print_help() {
    grep '^#' "$0" | sed 's/^# \{0,1\}//'
}

while [[ $# -gt 0 ]]; do
    case $1 in
        -t|--tracklet) TRACKLET_XML="$2"; shift 2;;
        -c|--config) RVIZ_CFG="$2"; shift 2;;
        -r|--rate) FRAME_RATE="$2"; shift 2;;
        --frame) FRAME_ID="$2"; shift 2;;
        --types) INCLUDE_TYPES="$2"; shift 2;;
        --no-text) ENABLE_TEXT=0; shift;;
        --no-traj) ENABLE_TRAJ=0; shift;;
        --traj-len) TRAJ_HISTORY_LEN="$2"; shift 2;;
        --traj-width) TRAJ_WIDTH="$2"; shift 2;;
        --coord) TRACKLET_COORD="$2"; shift 2;;
        --debug) DEBUG_MODE=1; shift;;
        --no-rviz) START_RVIZ=0; shift;;
        --sim-time) SET_SIM_TIME=1; shift;;
        --print-only) PRINT_ONLY=1; shift;;
        -h|--help) print_help; exit 0;;
        *) echo "未知参数: $1"; exit 1;;
    esac
done

# 1) 环境准备
if [[ ! -f catkin_ws/devel/setup.bash ]]; then
    echo "[错误] 找不到 catkin_ws/devel/setup.bash，请先编译 workspace" >&2
    exit 1
fi
source catkin_ws/devel/setup.bash

if [[ ! -f "$TRACKLET_XML" ]]; then
    echo "[错误] Tracklet 文件不存在: $TRACKLET_XML" >&2
    exit 1
fi
if [[ ! -f "$RVIZ_CFG" && $START_RVIZ -eq 1 ]]; then
    echo "[警告] RViz 配置不存在: $RVIZ_CFG (将使用默认空白配置)" >&2
    RVIZ_CFG=""
fi

ABS_TRACKLET=$(python3 -c 'import os,sys;print(os.path.abspath(sys.argv[1]))' "$TRACKLET_XML")

# 2) 启动 roscore (若未运行)
ROSCORE_STARTED=0
if ! pgrep -x roscore >/dev/null 2>&1; then
    echo "[信息] 启动 roscore..."
    roscore >/dev/null 2>&1 &
    ROSCORE_PID=$!
    ROSCORE_STARTED=1
    # 等待 roscore 可用
    for i in {1..20}; do
        if rosparam list >/dev/null 2>&1; then break; fi
        sleep 0.2
    done
else
    echo "[信息] 已检测到运行中的 roscore"
    ROSCORE_PID=""
fi

if [[ $SET_SIM_TIME -eq 1 ]]; then
    rosparam set use_sim_time true || true
fi

# 3) 组装发布命令 (使用 ROS 私有参数语法 _param:=value)
PUB_CMD=(rosrun kitti_tracklets_viz tracklets_publisher.py \
    _tracklet_file:="$ABS_TRACKLET" \
    _frame_id:="$FRAME_ID" \
    _frame_rate:="$FRAME_RATE" \
    _include_types:="$INCLUDE_TYPES" \
    _enable_text:="$ENABLE_TEXT" \
    _enable_traj:="$ENABLE_TRAJ" \
    _traj_history_len:="$TRAJ_HISTORY_LEN" \
    _traj_line_width:="$TRAJ_WIDTH" \
    _tracklet_coord:="$TRACKLET_COORD" \
    _velo_from_cam_R:="$VELO_FROM_CAM_R" \
    _velo_from_cam_t:="$VELO_FROM_CAM_T" \
    _debug:="$DEBUG_MODE" )

if [[ $PRINT_ONLY -eq 1 ]]; then
    echo "将执行的命令:"
    if [[ $ROSCORE_STARTED -eq 1 ]]; then echo "  roscore &"; fi
    echo "  ${PUB_CMD[*]}"
    if [[ $START_RVIZ -eq 1 ]]; then
        if [[ -n "$RVIZ_CFG" ]]; then
            echo "  rviz -d $RVIZ_CFG"
        else
            echo "  rviz"
        fi
    fi
    exit 0
fi

echo "[信息] 启动 tracklets 发布节点..."
"${PUB_CMD[@]}" &
PUBLISHER_PID=$!

# 4) 启动 RViz
if [[ $START_RVIZ -eq 1 ]]; then
    echo "[信息] 启动 RViz..."
    if [[ -n "$RVIZ_CFG" ]]; then
        rviz -d "$RVIZ_CFG" &
    else
        rviz &
    fi
    RVIZ_PID=$!
else
    RVIZ_PID=""
fi

# 5) 清理逻辑
cleanup() {
    echo -e "\n[清理] 捕获到退出信号，正在结束进程..."
    if [[ -n "${PUBLISHER_PID:-}" && -d /proc/$PUBLISHER_PID ]]; then
        kill $PUBLISHER_PID 2>/dev/null || true
    fi
    if [[ -n "${RVIZ_PID:-}" && -d /proc/$RVIZ_PID ]]; then
        kill $RVIZ_PID 2>/dev/null || true
    fi
    if [[ $ROSCORE_STARTED -eq 1 && -n "${ROSCORE_PID:-}" && -d /proc/$ROSCORE_PID ]]; then
        kill $ROSCORE_PID 2>/dev/null || true
    fi
    echo "[完成] 已退出。"
}
trap cleanup INT TERM EXIT

echo "[运行中] 可在 RViz 中查看: MarkerArray -> /kitti/tracklets_markers (命名空间: kitti_tracklets*)"
echo "[提示] 若未看到框: 1) 检查 Fixed Frame=$FRAME_ID 2) 取消相机远离 3) 确认时间是否在递增"
wait ${RVIZ_PID:-$PUBLISHER_PID}
