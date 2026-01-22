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
sleep 8  # 增加等待时间

# 2. 构建 catkin 工作空间
echo "[步骤 2] 正在构建/验证 catkin 工作空间..."
if ! "$PROJECT_ROOT/Scripts/utils/build_catkin_ws.sh"; then
    echo "[错误] 构建失败，正在中止..."
    exit 1
fi

# 确保 Results 目录存在
mkdir -p "$PROJECT_ROOT/Results"

# 4. 启动 PointPillars 节点
echo "[步骤 4] 正在启动 PointPillars 节点..."
"$PROJECT_ROOT/Scripts/pointpillars/run_pointpillars_node.sh" &
PP_PID=$!
sleep 15 # 给定充足时间加载模型 (通常需要10秒以上)

# 检查 PointPillars 是否存活
if ! kill -0 $PP_PID 2>/dev/null; then
    echo "[错误] PointPillars 节点启动失败"
    exit 1
fi

# 5. 启动 FAST-LIO 系统
echo "[步骤 5] 正在启动 FAST-LIO (建图 + 位姿桥接)..."
"$PROJECT_ROOT/Scripts/fastlio/run_fastlio.sh" both &
FL_PID=$!
sleep 5

# 检查 FastLIO 是否存活
if ! kill -0 $FL_PID 2>/dev/null; then
    echo "[错误] FastLIO 启动失败"
    exit 1
fi

# 6. 启动 MCTrack 在线节点
echo "[步骤 6] 正在启动 MCTrack 在线节点..."
"$PROJECT_ROOT/Scripts/mctrack/run_mctrack_online_node.sh" &
MC_PID=$!
sleep 3

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
    # 移除 --loop 选项以避免无限循环，从头播放一次以便评估
    rosbag play "$BAG_FILE" --clock &
    BAG_PID=$!
    
    echo "========================================================="
    echo "   流水线已运行！等待 Bag 播放结束...                    "
    echo "========================================================="
    
    # 等待 bag 播放器结束
    wait $BAG_PID
    echo "[信息] Bag 播放结束，等待 5 秒以确保所有数据处理完成..."
    sleep 5
fi

# 自动评估
echo "[步骤 9] 正在运行轨迹评估..."
python "$PROJECT_ROOT/Scripts/evaluation/evaluate_trajectory.py" \
    --pred "$PROJECT_ROOT/Results/trajectory.txt" \
    --gt "$PROJECT_ROOT/Data_Tracking/training/oxts/$SEQ_ID.txt" \
    --output "$PROJECT_ROOT/Results/"

echo "========================================================="
echo "   评估完成！结果保存在 Results/ 目录                    "
echo "========================================================="

# 如果跳过或结束了 bag 播放，保持脚本运行
wait
