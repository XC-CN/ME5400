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
    if [ "$SUCCESS" != "true" ]; then
        if kill -0 $RVIZ_PID 2>/dev/null; then kill $RVIZ_PID; fi
    fi
    if [ ! -z "$MC_PID" ] && kill -0 $MC_PID 2>/dev/null; then kill $MC_PID; fi
    if [ ! -z "$PUB_PID" ] && kill -0 $PUB_PID 2>/dev/null; then kill $PUB_PID; fi
}

# 解析参数
MODE="OPTIMIZED" # 默认模式
SEQ_ID="0020"

for arg in "$@"; do
    case $arg in
        -b|--baseline)
        MODE="BASELINE"
        shift
        ;;
        -h|--help)
            echo "用法: $0 [选项] [SEQ_ID]"
            echo "选项:"
            echo "  --baseline    仅运行基准测试 (无 MCTrack 优化)"
            echo "  -h, --help    显示帮助信息"
            echo ""
            echo "默认仅运行优化测试 (带 MCTrack 优化)"
            echo "示例:"
            echo "  $0            # 运行优化测试"
            echo "  $0 --baseline # 运行基准测试"
            exit 0
        ;;
        *)
        if [[ ! $arg =~ ^- ]]; then
            SEQ_ID=$arg
        fi
        ;;
    esac
done

# 捕获 SIGINT (Ctrl+C) 和 EXIT 信号
trap cleanup SIGINT EXIT

echo "[INFO] 正在启动 ME5400 完整流水线 (不包括步骤 3)..."
echo "[INFO] 数据集序列号: $SEQ_ID"
if [ "$MODE" = "BASELINE" ]; then
  echo "[INFO] 模式: 仅基准测试 (无优化)"
else
  echo "[INFO] 模式: 仅优化测试 (MCTrack 优化)"
fi


# 0. 环境清理
echo "[步骤 0] 正在清理残留环境..."
killall -9 rviz 2>/dev/null || true
killall -9 rosmaster 2>/dev/null || true
killall -9 roscore 2>/dev/null || true
killall -9 python 2>/dev/null | grep "ros" || true # 谨慎清理
sleep 1

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

# 确保 Results 和 PCD 目录存在
mkdir -p "$PROJECT_ROOT/Results"
mkdir -p "$PROJECT_ROOT/PCD"

# 3. 启动 RViz 可视化与真实的轨迹
echo "[步骤 3] 正在启动真实轨迹发布与 RViz..."

GT_FILE="$PROJECT_ROOT/Data_Tracking/training/oxts/${SEQ_ID}.txt"
if [[ -f "$GT_FILE" ]]; then
    python3 "$PROJECT_ROOT/Scripts/utils/publish_gt_path.py" "$GT_FILE" &
    PUB_PID=$!
else
    echo "[警告] 真实数据轨迹文件 $GT_FILE 未找到，将不会发布"
fi

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

# =========================================================
# 阶段 A: 基准测试 (可选)
# =========================================================
if [ "$MODE" = "BASELINE" ]; then
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

    echo "[步骤 7A] 正在播放 Rosbag (基准)..."
    BAG_FILE="$PROJECT_ROOT/Data_Tracking/rosbags/seq_${SEQ_ID}_nodet.bag"
    if [[ ! -f "$BAG_FILE" ]]; then
        echo "[错误] Bag 文件未找到: $BAG_FILE"
        exit 1
    fi

    rosparam set use_sim_time true
    rosbag play "$BAG_FILE" --clock &
    BAG_PID=$!

    echo "Waiting for Baseline Run..."
    wait $BAG_PID

    # 停止 FastLIO
    echo "[步骤 6A] 基准测试结束，停止 FastLIO..."
    kill -TERM $FL_PID 2>/dev/null || true
    wait $FL_PID 2>/dev/null || true
    
    # 保存结果并重命名
    if [[ -f "$PROJECT_ROOT/Results/trajectory.txt" ]]; then
        mv "$PROJECT_ROOT/Results/trajectory.txt" "$PROJECT_ROOT/Results/trajectory_baseline.txt"
        echo "[信息] 基准轨迹已保存为 Results/trajectory_baseline.txt"
    else
        echo "[警告] 未找到基准轨迹文件！"
    fi

    # 重命名地图文件 (防止被覆盖)
    if [[ -f "$PROJECT_ROOT/PCD/scans.pcd" ]]; then
        mv "$PROJECT_ROOT/PCD/scans.pcd" "$PROJECT_ROOT/PCD/scans_baseline.pcd"
        echo "[信息] 基准地图已保存为 PCD/scans_baseline.pcd"
    else
        echo "[警告] 未找到基准地图文件 (scans.pcd)"
    fi

    echo "[INFO] 基准测试完成！"

else
    # =========================================================
    # 阶段 B: 优化测试
    # =========================================================

    # 6. 启动/重启 MCTrack 在线节点
    # 如果 MC_PID 存在但进程不在运行，或者之前没启动
    if [ -z "$MC_PID" ] || ! kill -0 $MC_PID 2>/dev/null; then
         echo "[步骤 8] 正在启动 MCTrack 在线节点..."
        "$PROJECT_ROOT/Scripts/mctrack/run_mctrack_online_node.sh" &
        MC_PID=$!
        sleep 3
    fi

    echo "========================================================="
    echo "   阶段 B: 运行优化测试 (带 MCTrack 优化)                "
    echo "========================================================="

    echo "[步骤 6B] 正在启动 FAST-LIO (优化模式: use_dynamic_weights=true)..."
    "$PROJECT_ROOT/Scripts/fastlio/run_fastlio.sh" both use_dynamic_weights:=true &
    FL_PID=$!
    sleep 5

    # 7B. 播放 Rosbag (优化)
    BAG_FILE="$PROJECT_ROOT/Data_Tracking/rosbags/seq_${SEQ_ID}_nodet.bag"
    if [[ ! -f "$BAG_FILE" ]]; then
        echo "[错误] Bag 文件未找到: $BAG_FILE"
        exit 1
    fi

    echo "[步骤 7B] 正在播放 Rosbag (优化)..."
    rosparam set use_sim_time true
    rosbag play "$BAG_FILE" --clock &
    BAG_PID=$!

    echo "Waiting for Optimized Run..."
    wait $BAG_PID

    # 停止 FastLIO
    echo "[步骤 6B] 优化测试结束，停止 FastLIO..."
    kill -TERM $FL_PID 2>/dev/null || true
    wait $FL_PID 2>/dev/null || true
    echo "[信息] 优化轨迹已保存为 Results/trajectory.txt"

    # 重命名地图文件
    if [[ -f "$PROJECT_ROOT/PCD/scans.pcd" ]]; then
        mv "$PROJECT_ROOT/PCD/scans.pcd" "$PROJECT_ROOT/PCD/scans_optimized.pcd"
        echo "[信息] 优化地图已保存为 PCD/scans_optimized.pcd"
    else
        echo "[警告] 未找到优化地图文件 (scans.pcd)"
    fi

    # 自动评估 (仅当 baseline 存在时运行，或者修改评估脚本)
    if [ -f "$PROJECT_ROOT/Results/trajectory_baseline.txt" ]; then
        echo "[步骤 9] 正在运行增强轨迹评估..."
        python "$PROJECT_ROOT/Scripts/evaluation/evaluate_trajectory.py" \
            --pred "$PROJECT_ROOT/Results/trajectory.txt" \
            --baseline "$PROJECT_ROOT/Results/trajectory_baseline.txt" \
            --gt "$PROJECT_ROOT/Data_Tracking/training/oxts/$SEQ_ID.txt" \
            --output "$PROJECT_ROOT/Results/"
        
        echo "========================================================="
        echo "   评估完成！结果保存在 Results/ 目录                    "
        echo "========================================================="
    else
        echo "[信息] 未找到基准轨迹，跳过对比评估"
    fi
fi

# 主动停止所有后台进程
echo "[INFO] 任务完成，正在停止部分后台进程..."
kill $PP_PID 2>/dev/null || true
kill $MC_PID 2>/dev/null || true
kill $PUB_PID 2>/dev/null || true
kill $ROSCORE_PID 2>/dev/null || true

# 短暂等待进程退出
sleep 2

echo "[INFO] 所有进程已停止，但保留 RViz 继续运行，脚本正常退出"
SUCCESS=true
exit 0
