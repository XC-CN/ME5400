#!/bin/bash
set -e

# 获取项目根目录
SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
PROJECT_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
CATKIN_WS="$PROJECT_ROOT/catkin_ws"

source "$PROJECT_ROOT/Scripts/utils/setup_runtime_env.sh"

# 退出时处理清理工作的函数
cleanup() {
    if [ "$SUCCESS" != "true" ]; then
        echo -e "\n[INFO] 异常中止，正在清理残留进程..."
        if [ ! -z "$RVIZ_PID" ] && kill -0 $RVIZ_PID 2>/dev/null; then kill -9 $RVIZ_PID 2>/dev/null || true; fi
        if [ ! -z "$FL_PID" ] && kill -0 $FL_PID 2>/dev/null; then kill -9 $FL_PID 2>/dev/null || true; fi
        if [ ! -z "$BAG_PID" ] && kill -0 $BAG_PID 2>/dev/null; then kill -9 $BAG_PID 2>/dev/null || true; fi
        if [ ! -z "$PUB_PID" ] && kill -0 $PUB_PID 2>/dev/null; then kill -9 $PUB_PID 2>/dev/null || true; fi
        if [ "$INTERNAL_ROSCORE" = "true" ] && [ ! -z "$ROSCORE_PID" ] && kill -0 $ROSCORE_PID 2>/dev/null; then kill -9 $ROSCORE_PID 2>/dev/null || true; fi
    fi
}

# 解析参数
SEQ_ID="0020"
HEADLESS=true
SAVE_MAP=false

for arg in "$@"; do
    case $arg in
        -h|--help)
            echo "用法: $0 [选项] [SEQ_ID]"
            echo "选项:"
            echo "  -v, --viz       开启可视化模式 (加载 RViz)"
            echo "  -m, --save-map  启用 FAST-LIO 地图保存并归档结果"
            echo "  -h, --help      显示帮助信息"
            echo ""
            echo "纯净的单独 FAST-LIO 基准测试"
            echo "示例:"
            echo "  $0 0020         # 运行序列0020的基准测试 (默认无可视化)"
            echo "  $0 --viz 0020   # 开启 RViz 可视化运行"
            echo "  $0 --save-map 0020 # 运行并额外保存全局地图"
            exit 0
        ;;
        -v|--viz|--rviz)
            HEADLESS=false
        ;;
        -n|--headless|--no-rviz)
            HEADLESS=true
        ;;
        -m|--save-map)
            SAVE_MAP=true
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

echo "[INFO] 正在启动 ME5400 基准测试 (单独 FAST-LIO)..."
echo "[INFO] 数据集序列号: $SEQ_ID"
echo "[INFO] FAST-LIO 地图保存: $SAVE_MAP"

# 0. 环境清理
echo "[步骤 0] 正在清理残留环境..."
bash "$PROJECT_ROOT/Scripts/utils/cleanup_ros_runtime.sh"
sleep 1

# 1. 启动 roscore (条件启动)
if ! rostopic list > /dev/null 2>&1; then
    echo "[步骤 1] 正在启动 roscore..."
    roscore &
    ROSCORE_PID=$!
    sleep 8
    INTERNAL_ROSCORE=true
else
    echo "[步骤 1] 检测到 roscore 已在运行，将复用现有环境..."
    INTERNAL_ROSCORE=false
fi

# 2. 构建 catkin 工作空间
echo "[步骤 2] 正在构建/验证 catkin 工作空间..."
if ! "$PROJECT_ROOT/Scripts/utils/build_catkin_ws.sh"; then
    echo "[错误] 构建失败，正在中止..."
    exit 1
fi

source "$PROJECT_ROOT/Scripts/utils/setup_runtime_env.sh"

# 确保 Results 目录存在
mkdir -p "$PROJECT_ROOT/Results"

if [ "$HEADLESS" = "false" ]; then
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
else
    echo "[步骤 3] Headless 模式：跳过 RViz 可视化与真实轨迹发布..."
fi

# =========================================================
# 阶段 A: 基准测试 (单独 FAST-LIO)
# =========================================================

echo "========================================================="
echo "   运行基准测试 (单独 FAST-LIO)                          "
echo "========================================================="

echo "[步骤 4] 正在启动 FAST-LIO (基准模式: 纯 mapping, use_dynamic_weights=false)..."
# 注意：基准模式只启动原生 FAST-LIO mapping，
# 直接订阅 Rosbag 中的原始话题，不启动 pose_bridge / MCTrack / 检测中转链
FASTLIO_ARGS=(
    mapping
    use_dynamic_weights:=false
    lidar_topic:=/kitti/velo/pointcloud
    imu_topic:=/kitti/oxts/imu
)
if [[ "$SAVE_MAP" == "true" ]]; then
    FASTLIO_ARGS+=(pcd_save_en:=true)
fi
MAP_RUN_START_TS=$(date +%s)
"$PROJECT_ROOT/Scripts/fastlio/run_fastlio.sh" "${FASTLIO_ARGS[@]}" &
FL_PID=$!
sleep 5

# 检查 FastLIO 是否存活
if ! kill -0 $FL_PID 2>/dev/null; then
    echo "[错误] FastLIO (基准) 启动失败"
    exit 1
fi

echo "[步骤 5] 正在播放 Rosbag (基准)..."
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
echo "[步骤 6] 基准测试结束，停止 FastLIO..."
kill -TERM $FL_PID 2>/dev/null || true
wait $FL_PID 2>/dev/null || true

# 保存结果并重命名
mkdir -p "$PROJECT_ROOT/Results/${SEQ_ID}_results/Online"
if [[ -f "$PROJECT_ROOT/Results/trajectory.txt" ]]; then
    mv "$PROJECT_ROOT/Results/trajectory.txt" "$PROJECT_ROOT/Results/${SEQ_ID}_results/Online/trajectory_baseline.txt"
    echo "[信息] 基准轨迹已保存为 Results/${SEQ_ID}_results/Online/trajectory_baseline.txt"
else
    echo "[警告] 未找到基准轨迹文件！"
fi

# 如果本次显式开启了地图保存，则归档新生成的地图文件
if [[ "$SAVE_MAP" == "true" && -f "$PROJECT_ROOT/PCD/scans.pcd" ]]; then
    MAP_MTIME=$(stat -c %Y "$PROJECT_ROOT/PCD/scans.pcd")
    if [[ "$MAP_MTIME" -ge "$MAP_RUN_START_TS" ]]; then
        mv "$PROJECT_ROOT/PCD/scans.pcd" "$PROJECT_ROOT/Results/${SEQ_ID}_results/Online/scans_baseline.pcd"
        echo "[信息] 基准地图已保存为 Results/${SEQ_ID}_results/Online/scans_baseline.pcd"
        # 尝试删除临时的 PCD 目录
        rmdir "$PROJECT_ROOT/PCD" 2>/dev/null || true
    else
        echo "[警告] 检测到旧的 PCD 临时文件，未作为本次基准地图归档"
    fi
fi

echo "[INFO] 基准测试完成！"

# 如果优化后的轨迹也存在，自动触发定量对比评估
if [ -f "$PROJECT_ROOT/Results/${SEQ_ID}_results/Online/trajectory.txt" ] && [ -f "$PROJECT_ROOT/Results/${SEQ_ID}_results/Online/trajectory_baseline.txt" ]; then
    echo "[步骤 7] 正在运行增强轨迹评估（定量对比）..."
    python3 "$PROJECT_ROOT/Scripts/evaluation/evaluate_trajectories.py" \
        --gt "$PROJECT_ROOT/Data_Tracking/training/oxts/$SEQ_ID.txt" \
        --traj "Baseline:$PROJECT_ROOT/Results/${SEQ_ID}_results/Online/trajectory_baseline.txt" \
        --traj "Optimized:$PROJECT_ROOT/Results/${SEQ_ID}_results/Online/trajectory.txt" \
        --output-dir "$PROJECT_ROOT/Results/${SEQ_ID}_results/Online/" \
        --report-name "metrics.txt" \
        --report-json-name "metrics.json" \
        --report-title "在线轨迹评估报告" \
        --pair-plot-name "evaluation_result.png"
    
    echo "========================================================="
    echo "   评估完成！定量对比结果保存在 Results/${SEQ_ID}_results/Online/ 目录"
    echo "========================================================="
fi

# 主动停止所有后台进程
echo "[INFO] 任务完成，正在停止部分后台进程..."
[ ! -z "$PUB_PID" ] && kill $PUB_PID 2>/dev/null || true
[ ! -z "$ROSCORE_PID" ] && kill $ROSCORE_PID 2>/dev/null || true

# 短暂等待进程退出
sleep 2

if [ "$HEADLESS" = "false" ]; then
    echo "[INFO] 所有进程已停止，但保留 RViz 继续运行，脚本正常退出"
else
    echo "[INFO] 所有进程已停止，脚本正常退出"
fi
SUCCESS=true
exit 0
