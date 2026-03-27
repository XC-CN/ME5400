#!/bin/bash
set -e

# 获取项目根目录
SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
PROJECT_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)

# 退出时处理清理工作的函数
cleanup() {
    if [ "$SUCCESS" != "true" ]; then
        echo -e "\n[INFO] 异常中止，正在清理残留进程..."
        if [ ! -z "$RVIZ_PID" ] && kill -0 $RVIZ_PID 2>/dev/null; then kill -9 $RVIZ_PID 2>/dev/null || true; fi
        if [ ! -z "$FL_PID" ] && kill -0 $FL_PID 2>/dev/null; then kill -9 $FL_PID 2>/dev/null || true; fi
        if [ ! -z "$BAG_PID" ] && kill -0 $BAG_PID 2>/dev/null; then kill -9 $BAG_PID 2>/dev/null || true; fi
        if [ ! -z "$PUB_PID" ] && kill -0 $PUB_PID 2>/dev/null; then kill -9 $PUB_PID 2>/dev/null || true; fi
        if [ ! -z "$ROSCORE_PID" ] && kill -0 $ROSCORE_PID 2>/dev/null; then kill -9 $ROSCORE_PID 2>/dev/null || true; fi
    fi
}

# 解析参数
SEQ_ID="0020"
HEADLESS=false

for arg in "$@"; do
    case $arg in
        -h|--help)
            echo "用法: $0 [选项] [SEQ_ID]"
            echo "选项:"
            echo "  -n, --headless  无可视化模式 (跳过加载 RViz)"
            echo "  -h, --help      显示帮助信息"
            echo ""
            echo "纯净的单独 FAST-LIO 基准测试"
            echo "示例:"
            echo "  $0 0020         # 运行序列0020的基准测试"
            echo "  $0 --headless 0020 # 以 Headless 模式运行"
            exit 0
        ;;
        -n|--headless|--no-rviz)
            HEADLESS=true
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

echo "[步骤 4] 正在启动 FAST-LIO (基准模式: use_dynamic_weights=false)..."
# 注意：基准模式直接订阅 Rosbag 中的原始话题，而不通过检测节点中转
"$PROJECT_ROOT/Scripts/fastlio/run_fastlio.sh" both \
    use_dynamic_weights:=false \
    lidar_topic:=/kitti/velo/pointcloud \
    imu_topic:=/kitti/oxts/imu &
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
mkdir -p "$PROJECT_ROOT/Results/${SEQ_ID}_results"
if [[ -f "$PROJECT_ROOT/Results/trajectory.txt" ]]; then
    mv "$PROJECT_ROOT/Results/trajectory.txt" "$PROJECT_ROOT/Results/${SEQ_ID}_results/trajectory_baseline.txt"
    echo "[信息] 基准轨迹已保存为 Results/${SEQ_ID}_results/trajectory_baseline.txt"
else
    echo "[警告] 未找到基准轨迹文件！"
fi

# 如果配置中开启了 pcd_save_en，则重命名地图文件并移入结果目录
if [[ -f "$PROJECT_ROOT/PCD/scans.pcd" ]]; then
    mv "$PROJECT_ROOT/PCD/scans.pcd" "$PROJECT_ROOT/Results/${SEQ_ID}_results/scans_baseline.pcd"
    # 尝试删除临时的 PCD 目录
    rmdir "$PROJECT_ROOT/PCD" 2>/dev/null || true
fi

echo "[INFO] 基准测试完成！"

# 如果优化后的轨迹也存在，自动触发定量对比评估
if [ -f "$PROJECT_ROOT/Results/${SEQ_ID}_results/trajectory.txt" ] && [ -f "$PROJECT_ROOT/Results/${SEQ_ID}_results/trajectory_baseline.txt" ]; then
    echo "[步骤 7] 正在运行增强轨迹评估（定量对比）..."
    python "$PROJECT_ROOT/Scripts/evaluation/evaluate_trajectory.py" \
        --pred "$PROJECT_ROOT/Results/${SEQ_ID}_results/trajectory.txt" \
        --baseline "$PROJECT_ROOT/Results/${SEQ_ID}_results/trajectory_baseline.txt" \
        --gt "$PROJECT_ROOT/Data_Tracking/training/oxts/$SEQ_ID.txt" \
        --output "$PROJECT_ROOT/Results/"
    
    echo "========================================================="
    echo "   评估完成！定量对比结果保存在 Results/${SEQ_ID}_results/ 目录"
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
