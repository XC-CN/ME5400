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
        if [ ! -z "$PP_PID" ] && kill -0 $PP_PID 2>/dev/null; then kill -9 $PP_PID 2>/dev/null || true; fi
        if [ ! -z "$MC_PID" ] && kill -0 $MC_PID 2>/dev/null; then kill -9 $MC_PID 2>/dev/null || true; fi
        if [ ! -z "$FL_PID" ] && kill -0 $FL_PID 2>/dev/null; then kill -9 $FL_PID 2>/dev/null || true; fi
        if [ ! -z "$BAG_PID" ] && kill -0 $BAG_PID 2>/dev/null; then kill -9 $BAG_PID 2>/dev/null || true; fi
        if [ ! -z "$PUB_PID" ] && kill -0 $PUB_PID 2>/dev/null; then kill -9 $PUB_PID 2>/dev/null || true; fi
        if [ "$INTERNAL_ROSCORE" = "true" ] && [ ! -z "$ROSCORE_PID" ] && kill -0 $ROSCORE_PID 2>/dev/null; then kill -9 $ROSCORE_PID 2>/dev/null || true; fi
    fi
}

# 解析参数
SEQ_ID="0020"
HEADLESS=false
SAVE_MAP=false

for arg in "$@"; do
    case $arg in
        -h|--help)
            echo "用法: $0 [选项] [SEQ_ID]"
            echo "选项:"
            echo "  -n, --headless  无可视化模式 (跳过加载 RViz)"
            echo "  -m, --save-map  启用 FAST-LIO 地图保存并归档结果"
            echo "  -h, --help      显示帮助信息"
            echo ""
            echo "运行带 MCTrack 的优化流水线"
            echo "示例:"
            echo "  $0 0020         # 运行序列0020的优化测试"
            echo "  $0 --headless 0020 # 以 Headless 模式运行"
            echo "  $0 --save-map 0020 # 运行并额外保存全局地图"
            exit 0
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

echo "[INFO] 正在启动 ME5400 完整优化流水线..."
echo "[INFO] 数据集序列号: $SEQ_ID"
echo "[INFO] FAST-LIO 地图保存: $SAVE_MAP"
if [[ -n "${PP_ALLOWED_CLASSES:-}" ]]; then
    echo "[INFO] PointPillars 类别过滤: $PP_ALLOWED_CLASSES"
fi
if [[ -n "${PP_CONFIG_PATH:-}" ]]; then
    echo "[INFO] PointPillars 配置覆盖: $PP_CONFIG_PATH"
fi
if [[ -n "${PP_CHECKPOINT_PATH:-}" ]]; then
    echo "[INFO] PointPillars 权重覆盖: $PP_CHECKPOINT_PATH"
fi

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
# 阶段 B: 优化测试
# =========================================================

# 4. 启动 PointPillars 节点 (清除 BAG_FILE 防止其内部触发播放, 传递序列号以便加载正确标定)
echo "[步骤 4] 正在启动 PointPillars 节点 (Sequence: $SEQ_ID)..."
PP_NODE_ARGS=()
if [[ -n "${PP_ALLOWED_CLASSES:-}" ]]; then
    PP_NODE_ARGS+=("_allowed_classes:=$PP_ALLOWED_CLASSES")
fi
if [[ -n "${PP_CONFIG_PATH:-}" ]]; then
    PP_NODE_ARGS+=("_config_path:=$PP_CONFIG_PATH")
fi
if [[ -n "${PP_CHECKPOINT_PATH:-}" ]]; then
    PP_NODE_ARGS+=("_checkpoint_path:=$PP_CHECKPOINT_PATH")
fi
BAG_FILE="" "$PROJECT_ROOT/Scripts/pointpillars/run_pointpillars_node.sh" _seq:="$SEQ_ID" "${PP_NODE_ARGS[@]}" &
PP_PID=$!
sleep 15 

# 5. 启动 MCTrack 在线节点
echo "[步骤 5] 正在启动 MCTrack 在线节点..."
"$PROJECT_ROOT/Scripts/mctrack/run_mctrack_online_node.sh" &
MC_PID=$!
sleep 3

echo "========================================================="
echo "   运行优化测试 (带 MCTrack 优化)                        "
echo "========================================================="

echo "[步骤 6] 正在启动 FAST-LIO (优化模式: use_dynamic_weights=true)..."
FASTLIO_ARGS=(
    both
    use_dynamic_weights:=true
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

BAG_FILE_PATH="$PROJECT_ROOT/Data_Tracking/rosbags/seq_${SEQ_ID}_nodet.bag"
if [[ ! -f "$BAG_FILE_PATH" ]]; then
    echo "[错误] Bag 文件未找到: $BAG_FILE_PATH"
    exit 1
fi

echo "[步骤 7] 正在播放 Rosbag (优化, 以 1.0 倍速运行，跳过 bag 内 /tf_static)..."
rosparam set use_sim_time true
# 仅播放原始 IMU / 点云，避免 bag 内 imu->velodyne 静态 TF
# 与 fastlio_pose_bridge 发布的 camera_init->velodyne 动态 TF 冲突，
# 进而导致 RViz 中原始点云前后抽动。
rosbag play "$BAG_FILE_PATH" --clock --topics /kitti/velo/pointcloud /kitti/oxts/imu &
BAG_PID=$!

echo "Waiting for Optimized Run..."
wait $BAG_PID

# 停止 FastLIO
echo "[步骤 8] 优化测试结束，停止 FastLIO..."
kill -TERM $FL_PID 2>/dev/null || true
wait $FL_PID 2>/dev/null || true

# 保存结果并重命名
mkdir -p "$PROJECT_ROOT/Results/${SEQ_ID}_results/Online"
if [[ -f "$PROJECT_ROOT/Results/trajectory.txt" ]]; then
    mv "$PROJECT_ROOT/Results/trajectory.txt" "$PROJECT_ROOT/Results/${SEQ_ID}_results/Online/trajectory.txt"
    echo "[信息] 优化轨迹已保存为 Results/${SEQ_ID}_results/Online/trajectory.txt"
fi

# 如果本次显式开启了地图保存，则归档新生成的地图文件
if [[ "$SAVE_MAP" == "true" && -f "$PROJECT_ROOT/PCD/scans.pcd" ]]; then
    MAP_MTIME=$(stat -c %Y "$PROJECT_ROOT/PCD/scans.pcd")
    if [[ "$MAP_MTIME" -ge "$MAP_RUN_START_TS" ]]; then
        mv "$PROJECT_ROOT/PCD/scans.pcd" "$PROJECT_ROOT/Results/${SEQ_ID}_results/Online/scans_optimized.pcd"
        echo "[信息] 优化地图已保存为 Results/${SEQ_ID}_results/Online/scans_optimized.pcd"
        # 尝试删除临时的 PCD 目录
        rmdir "$PROJECT_ROOT/PCD" 2>/dev/null || true
    else
        echo "[警告] 检测到旧的 PCD 临时文件，未作为本次优化地图归档"
    fi
fi

# 自动评估 (仅当 baseline 存在时运行，或者修改评估脚本)
if [ -f "$PROJECT_ROOT/Results/${SEQ_ID}_results/Online/trajectory_baseline.txt" ]; then
    echo "[步骤 9] 正在运行增强轨迹评估..."
    python "$PROJECT_ROOT/Scripts/evaluation/evaluate_trajectory.py" \
        --pred "$PROJECT_ROOT/Results/${SEQ_ID}_results/Online/trajectory.txt" \
        --baseline "$PROJECT_ROOT/Results/${SEQ_ID}_results/Online/trajectory_baseline.txt" \
        --gt "$PROJECT_ROOT/Data_Tracking/training/oxts/$SEQ_ID.txt" \
        --output "$PROJECT_ROOT/Results/${SEQ_ID}_results/Online/"
    
    echo "========================================================="
    echo "   评估完成！结果保存在 Results/${SEQ_ID}_results/Online/ 目录  "
    echo "========================================================="
else
    echo "[信息] 未找到基准轨迹，跳过对比评估"
fi

# 主动停止所有后台进程
echo "[INFO] 任务完成，正在停止部分后台进程..."
[ ! -z "$PP_PID" ] && kill $PP_PID 2>/dev/null || true
[ ! -z "$MC_PID" ] && kill $MC_PID 2>/dev/null || true
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
