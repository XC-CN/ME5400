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
    if kill -0 $RVIZ_PID 2>/dev/null; then kill $RVIZ_PID; fi
    if [ ! -z "$MC_PID" ] && kill -0 $MC_PID 2>/dev/null; then kill $MC_PID; fi
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

# 3. 启动 RViz 可视化
echo "[步骤 3] 正在启动 RViz..."
"$PROJECT_ROOT/Scripts/rviz/run_rviz.sh" &
RVIZ_PID=$!


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

# 5. 启动 MCTrack 节点 (第一次启动)
echo "[步骤 5] 正在启动 MCTrack 在线节点..."
"$PROJECT_ROOT/Scripts/mctrack/run_mctrack_online_node.sh" &
MC_PID=$!
sleep 2

# 6. 启动 FAST-LIO 系统 (第一遍：基准测试 - 无 MCTrack 优化)
echo "========================================================="
echo "   阶段 A: 运行基准测试 (无 MCTrack 优化)                "
echo "========================================================="

echo "[步骤 6A] 正在启动 FAST-LIO (基准模式: use_dynamic_weights=false)..."
"$PROJECT_ROOT/Scripts/fastlio/run_fastlio.sh" both use_dynamic_weights:=false &
FL_PID=$!
sleep 5

# 检查 FastLIO 是否存活
if ! kill -0 $FL_PID 2>/dev/null; then
    echo "[错误] FastLIO (基准) 启动失败"
    exit 1
fi

# 7A. 播放 Rosbag (基准)
SEQ_ID=${1:-"0020"}
BAG_FILE="$PROJECT_ROOT/Data_Tracking/rosbags/seq_${SEQ_ID}_nodet.bag"

if [[ ! -f "$BAG_FILE" ]]; then
    echo "[错误] Bag 文件未找到: $BAG_FILE"
    exit 1
fi

echo "[步骤 7A] 正在播放 Rosbag (基准)..."
rosparam set use_sim_time true
rosbag play "$BAG_FILE" --clock &
BAG_PID=$!

echo "Waiting for Baseline Run..."
wait $BAG_PID

# 停止 FastLIO 并重命名轨迹文件
echo "[步骤 6A] 基准测试结束，停止 FastLIO..."
kill -TERM $FL_PID 2>/dev/null || true
wait $FL_PID 2>/dev/null || true
# 等待文件写入完成 (fastlio 脚本内已处理等待)

if [[ -f "$PROJECT_ROOT/Results/trajectory.txt" ]]; then
    mv "$PROJECT_ROOT/Results/trajectory.txt" "$PROJECT_ROOT/Results/trajectory_baseline.txt"
    echo "[信息] 基准轨迹已保存为 Results/trajectory_baseline.txt"
else
    echo "[警告] 未找到基准轨迹文件！"
fi

# 重置 FastLIO 状态或稍作等待

# 5. 启动 FAST-LIO 系统 (第二遍：优化测试 - 有 MCTrack 优化)
echo "========================================================="
echo "   阶段 B: 运行优化测试 (带 MCTrack 优化)                "
echo "========================================================="

# 6. 重启 MCTrack 在线节点 (重置状态)
if kill -0 $MC_PID 2>/dev/null; then
    echo "[信息] 重启 MCTrack 以重置状态..."
    kill $MC_PID 2>/dev/null || true
    wait $MC_PID 2>/dev/null || true
fi

echo "[步骤 8] 正在启动 MCTrack 在线节点..."
"$PROJECT_ROOT/Scripts/mctrack/run_mctrack_online_node.sh" &
MC_PID=$!
sleep 3

echo "[步骤 6B] 正在启动 FAST-LIO (优化模式: use_dynamic_weights=true)..."
"$PROJECT_ROOT/Scripts/fastlio/run_fastlio.sh" both use_dynamic_weights:=true &
FL_PID=$!
sleep 5

# 7B. 播放 Rosbag (优化)
echo "[步骤 7B] 正在播放 Rosbag (优化)..."
rosparam set use_sim_time true
rosbag play "$BAG_FILE" --clock &
BAG_PID=$!

echo "Waiting for Optimized Run..."
wait $BAG_PID

# 停止 FastLIO
echo "[步骤 5B] 优化测试结束，停止 FastLIO..."
kill -TERM $FL_PID 2>/dev/null || true
wait $FL_PID 2>/dev/null || true
echo "[信息] 优化轨迹已保存为 Results/trajectory.txt"

# 自动评估
echo "[步骤 9] 正在运行增强轨迹评估..."
python "$PROJECT_ROOT/Scripts/evaluation/evaluate_trajectory.py" \
    --pred "$PROJECT_ROOT/Results/trajectory.txt" \
    --baseline "$PROJECT_ROOT/Results/trajectory_baseline.txt" \
    --gt "$PROJECT_ROOT/Data_Tracking/training/oxts/$SEQ_ID.txt" \
    --output "$PROJECT_ROOT/Results/"


echo "========================================================="
echo "   评估完成！结果保存在 Results/ 目录                    "
echo "========================================================="

# 如果跳过或结束了 bag 播放，保持脚本运行
wait
