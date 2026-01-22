#!/bin/bash
set -e

# 获取项目根目录
SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
PROJECT_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)

# 退出时处理清理工作的函数
cleanup() {
    echo -e "\n[INFO] 正在停止所有进程..."
    # 杀死同一进程组中的所有子进程
    trap - SIGTERM && kill -- -$$
}

# 捕获 SIGINT (Ctrl+C) 和 EXIT 信号
trap cleanup SIGINT EXIT

echo "[INFO] 正在启动 ME5400 完整流水线 (不包括步骤 3)..."

# 1. 启动 roscore
echo "[步骤 1] 正在启动 roscore..."
roscore &
ROSCORE_PID=$!
sleep 5  # 等待 roscore 初始化

# 2. 构建 catkin 工作空间 (如果已构建则速度很快)
echo "[步骤 2] 正在构建/验证 catkin 工作空间..."
"$PROJECT_ROOT/Scripts/utils/build_catkin_ws.sh"

# 4. 启动 PointPillars 节点
echo "[步骤 4] 正在启动 PointPillars 节点..."
"$PROJECT_ROOT/Scripts/pointpillars/run_pointpillars_node.sh" &
PP_PID=$!
sleep 5 # 等待模型加载

# 5. 启动 FAST-LIO 系统
echo "[步骤 5] 正在启动 FAST-LIO (建图 + 位姿桥接)..."
"$PROJECT_ROOT/Scripts/fastlio/run_fastlio.sh" both &
FL_PID=$!
sleep 3

# 6. 启动 MCTrack 在线节点
echo "[步骤 6] 正在启动 MCTrack 在线节点..."
"$PROJECT_ROOT/Scripts/mctrack/run_mctrack_online_node.sh" &
MC_PID=$!
sleep 2

# 8. 启动 RViz
echo "[步骤 8] 正在启动 RViz..."
"$PROJECT_ROOT/Scripts/rviz/run_rviz.sh" &
RVIZ_PID=$!
sleep 5

# 7. 播放 Rosbag
# 默认序列为 0020
SEQ_ID=${1:-"0020"}
BAG_FILE="$PROJECT_ROOT/Data_Tracking/rosbags/seq_${SEQ_ID}_nodet.bag"

if [[ ! -f "$BAG_FILE" ]]; then
    echo "[警告] 在 $BAG_FILE 未找到 Bag 文件"
    echo "       请检查序列 ID '$SEQ_ID' 是否正确，以及 bag 是否存在于 Data_Tracking/rosbags/ 中"
    echo "       将在不播放 bag 的情况下运行 (需要手动播放)。"
    wait $RVIZ_PID
else
    echo "[步骤 7] 正在播放 Rosbag: $BAG_FILE"
    rosparam set use_sim_time true
    rosbag play "$BAG_FILE" --clock --loop &
    BAG_PID=$!
    
    echo "========================================================="
    echo "   流水线已运行！按 Ctrl+C 停止所有进程。                "
    echo "========================================================="
    
    # 等待 bag 播放器或用户中断
    wait $BAG_PID
fi

# 如果跳过或结束了 bag 播放，保持脚本运行
wait
