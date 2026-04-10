#!/bin/bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
PROJECT_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
CATKIN_WS="$PROJECT_ROOT/catkin_ws"
OFFLINE_FEEDER_CONFIG="$CATKIN_WS/src/ME5400/config/offline_bag_feeder.yaml"
JOINT_GATED_POINTCLOUD_TOPIC="/offline/detection_pointcloud"
CONDA_SH="${HOME}/miniconda3/etc/profile.d/conda.sh"
CONDA_ENV_NAME="ME5400"

SUCCESS=false
HEADLESS=true
SAVE_MAP=false
SEQ_ID="0020"
BAG_PATH=""
JOINT_BACKEND_CONFIG_PATH="$PROJECT_ROOT/catkin_ws/src/ME5400/config/joint_backend_ego.yaml"

ROSCORE_PID=""
INTERNAL_ROSCORE=false
RVIZ_PID=""
PUB_PID=""

PP_PID=""
MC_PID=""
FL_PID=""
JB_PID=""
REC_PID=""
FEEDER_PID=""

usage() {
    cat <<EOF
用法: $0 [选项] [SEQ_ID]

选项:
  -v, --viz        开启可视化模式 (加载 RViz)
  -m, --save-map   为离线双阶段都启用 FAST-LIO 地图保存并归档
  --bag <path>     覆盖默认 bag 路径
  --jb-config <p>  覆盖 joint backend 配置文件路径
  -h, --help       显示帮助

说明:
  一键执行离线双轨实验：
  1) Baseline(Offline,NoWeight)
  2) JointOffline (PointPillars + MCTrack + Dynamic Weights + Joint Backend)

输出:
  Results/<SEQ_ID>_results/trajectory_baseline_offline.txt
  Results/<SEQ_ID>_results/trajectory_weighted_offline.txt
  Results/<SEQ_ID>_results/trajectory_joint.txt
  Results/<SEQ_ID>_results/metrics.txt

示例:
  $0 0020
  $0 --viz 0020 (开启可视化运行)
  $0 --bag Data_Tracking/rosbags/seq_0020_nodet.bag 0020
EOF
}

# 快速检查帮助信息，避免 source ROS 环境时将 --help 传递给 setup_util
for arg in "$@"; do
    case "$arg" in
        -h|--help)
            usage
            exit 0
            ;;
    esac
done

source "$PROJECT_ROOT/Scripts/utils/setup_runtime_env.sh"

while [[ $# -gt 0 ]]; do
    case "$1" in
        -h|--help)
            usage
            exit 0
            ;;
        -v|--viz|--rviz)
            HEADLESS=false
            shift
            ;;
        -n|--headless|--no-rviz)
            HEADLESS=true
            shift
            ;;
        -m|--save-map)
            SAVE_MAP=true
            shift
            ;;
        --bag)
            BAG_PATH="$2"
            shift 2
            ;;
        --jb-config)
            JOINT_BACKEND_CONFIG_PATH="$2"
            shift 2
            ;;
        *)
            if [[ "$1" =~ ^- ]]; then
                echo "[错误] 未知参数: $1" >&2
                usage >&2
                exit 1
            fi
            SEQ_ID="$1"
            shift
            ;;
    esac
done

RESULT_DIR="$PROJECT_ROOT/Results/${SEQ_ID}_results/Offline"
mkdir -p "$RESULT_DIR"
LOG_FILE="$RESULT_DIR/full_run_offline.log"
exec > >(tee -a "$LOG_FILE") 2>&1

stop_pid() {
    local pid="$1"
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
        kill -TERM "$pid" 2>/dev/null || true
        wait "$pid" 2>/dev/null || true
    fi
}

kill_matching_process() {
    local pattern="$1"
    pkill -TERM -f "$pattern" 2>/dev/null || true
}

cleanup_stage_processes() {
    stop_pid "$FEEDER_PID"
    FEEDER_PID=""

    stop_pid "$REC_PID"
    REC_PID=""
    kill_matching_process "odom_to_tum_recorder.py"

    stop_pid "$JB_PID"
    JB_PID=""
    kill_matching_process "joint_backend_ego_node.py"

    stop_pid "$FL_PID"
    FL_PID=""
    kill_matching_process "mapping_velodyne.launch"
    kill_matching_process "fastlio_mapping"
    kill_matching_process "laserMapping"
    kill_matching_process "fastlio_pose_bridge.py"

    stop_pid "$MC_PID"
    MC_PID=""
    kill_matching_process "mctrack_online_node.py"

    stop_pid "$PP_PID"
    PP_PID=""
    kill_matching_process "kitti_pointpillars_bag_node.py"
}

cleanup() {
    if [[ "$SUCCESS" != "true" ]]; then
        echo
        echo "[INFO] 异常中止，正在清理离线双轨进程..."
        cleanup_stage_processes
        stop_pid "$PUB_PID"
        PUB_PID=""
        stop_pid "$RVIZ_PID"
        RVIZ_PID=""
        if [[ "$INTERNAL_ROSCORE" == "true" ]]; then
            stop_pid "$ROSCORE_PID"
            ROSCORE_PID=""
        fi
    fi
}

trap cleanup SIGINT EXIT

resolve_path() {
    local raw="$1"
    if [[ "$raw" == /* ]]; then
        printf '%s\n' "$raw"
        return
    fi
    if [[ -f "$PROJECT_ROOT/$raw" ]]; then
        printf '%s\n' "$PROJECT_ROOT/$raw"
        return
    fi
    printf '%s\n' "$raw"
}

if [[ -n "$BAG_PATH" ]]; then
    BAG_FILE_PATH=$(resolve_path "$BAG_PATH")
else
    BAG_FILE_PATH="$PROJECT_ROOT/Data_Tracking/rosbags/seq_${SEQ_ID}_nodet.bag"
fi

if [[ ! -f "$BAG_FILE_PATH" ]]; then
    echo "[错误] Bag 文件未找到: $BAG_FILE_PATH" >&2
    exit 1
fi

JOINT_BACKEND_CONFIG_PATH=$(resolve_path "$JOINT_BACKEND_CONFIG_PATH")
if [[ ! -f "$JOINT_BACKEND_CONFIG_PATH" ]]; then
    echo "[错误] joint backend 配置未找到: $JOINT_BACKEND_CONFIG_PATH" >&2
    exit 1
fi

GT_PATH="$PROJECT_ROOT/Data_Tracking/training/oxts/${SEQ_ID}.txt"
CALIB_PATH="$PROJECT_ROOT/Data_Tracking/training/calib/${SEQ_ID}.txt"

if [[ ! -f "$GT_PATH" ]]; then
    echo "[错误] GT 文件未找到: $GT_PATH" >&2
    exit 1
fi

if [[ ! -f "$CALIB_PATH" ]]; then
    echo "[错误] 标定文件未找到: $CALIB_PATH" >&2
    exit 1
fi

echo "[INFO] 本次运行的所有输出将实时保存至: $LOG_FILE"
echo "[INFO] 数据集序列号: $SEQ_ID"
echo "[INFO] 离线 bag: $BAG_FILE_PATH"
echo "[INFO] FAST-LIO 地图保存: $SAVE_MAP"
echo "[INFO] Joint backend 配置: $JOINT_BACKEND_CONFIG_PATH"

echo "[步骤 0] 正在清理残留环境..."
bash "$PROJECT_ROOT/Scripts/utils/cleanup_ros_runtime.sh"
sleep 1

if ! rostopic list > /dev/null 2>&1; then
    echo "[步骤 1] 正在启动 roscore..."
    roscore &
    ROSCORE_PID=$!
    INTERNAL_ROSCORE=true
    sleep 8
else
    echo "[步骤 1] 检测到 roscore 已在运行，将复用现有环境..."
    INTERNAL_ROSCORE=false
fi

echo "[步骤 2] 正在构建/验证 catkin 工作空间..."
if ! "$PROJECT_ROOT/Scripts/utils/build_catkin_ws.sh"; then
    echo "[错误] 构建失败，正在中止..."
    exit 1
fi

source "$PROJECT_ROOT/Scripts/utils/setup_runtime_env.sh"

if [[ "$HEADLESS" == "false" ]]; then
    echo "[步骤 3] 正在启动真实轨迹发布与 RViz..."
    python3 "$PROJECT_ROOT/Scripts/utils/publish_gt_path.py" "$GT_PATH" &
    PUB_PID=$!
    "$PROJECT_ROOT/Scripts/rviz/run_rviz.sh" &
    RVIZ_PID=$!
else
    echo "[步骤 3] Headless 模式：跳过 RViz 可视化与真实轨迹发布..."
fi

archive_trajectory_if_new() {
    local stage_start_ts="$1"
    local dest_path="$2"
    local src_path="$PROJECT_ROOT/Results/trajectory.txt"

    if [[ ! -f "$src_path" ]]; then
        echo "[警告] 未找到轨迹文件: $src_path"
        return
    fi

    local traj_mtime
    traj_mtime=$(stat -c %Y "$src_path")
    if [[ "$traj_mtime" -lt "$stage_start_ts" ]]; then
        echo "[警告] 检测到旧的 trajectory.txt，未归档到 $(basename "$dest_path")"
        return
    fi

    mv "$src_path" "$dest_path"
    echo "[信息] 轨迹已保存为 ${dest_path#$PROJECT_ROOT/}"
}

archive_map_if_new() {
    local stage_start_ts="$1"
    local dest_path="$2"
    local src_path="$PROJECT_ROOT/PCD/scans.pcd"

    if [[ "$SAVE_MAP" != "true" || ! -f "$src_path" ]]; then
        return
    fi

    local map_mtime
    map_mtime=$(stat -c %Y "$src_path")
    if [[ "$map_mtime" -lt "$stage_start_ts" ]]; then
        echo "[警告] 检测到旧的 PCD 临时文件，未归档到 $(basename "$dest_path")"
        return
    fi

    mv "$src_path" "$dest_path"
    echo "[信息] 地图已保存为 ${dest_path#$PROJECT_ROOT/}"
    rmdir "$PROJECT_ROOT/PCD" 2>/dev/null || true
}

start_offline_feeder() {
    local require_detection="$1"
    local require_tracking="$2"
    local require_odom="${3:-false}"
    local gated_lidar_topic="${4:-}"
    local odom_topic="${5:-/joint_backend/odom}"
    local stamp_tolerance="${6:-0.06}"
    local gated_lidar_display="$gated_lidar_topic"
    if [[ -z "$gated_lidar_display" ]]; then
        gated_lidar_display="/kitti/velo/pointcloud"
    fi

    rosparam set use_sim_time true
    rosparam load "$OFFLINE_FEEDER_CONFIG" /offline_bag_feeder
    rosparam set /offline_bag_feeder/bag_path "$BAG_FILE_PATH"
    rosparam set /offline_bag_feeder/require_detection "$require_detection"
    rosparam set /offline_bag_feeder/require_tracking "$require_tracking"
    rosparam set /offline_bag_feeder/require_odom "$require_odom"
    rosparam set /offline_bag_feeder/gated_lidar_topic "$gated_lidar_topic"
    rosparam set /offline_bag_feeder/odom_topic "$odom_topic"
    rosparam set /offline_bag_feeder/stamp_tolerance "$stamp_tolerance"

    echo "[信息] 启动 offline_bag_feeder (require_detection=$require_detection require_tracking=$require_tracking require_odom=$require_odom odom_topic=$odom_topic stamp_tolerance=$stamp_tolerance gated_lidar_topic=$gated_lidar_display)"
    rosrun ME5400 offline_bag_feeder.py __name:=offline_bag_feeder &
    FEEDER_PID=$!
}

run_baseline_offline() {
    local stage_start_ts
    stage_start_ts=$(date +%s)

    echo
    echo "========================================================="
    echo "   [离线阶段 A] Baseline(Offline,NoWeight)"
    echo "========================================================="

    echo "[A-1] 启动 FAST-LIO (mapping, use_dynamic_weights=false)..."
    FASTLIO_ARGS=(
        mapping
        use_dynamic_weights:=false
        lidar_topic:=/kitti/velo/pointcloud
        imu_topic:=/kitti/oxts/imu
    )
    if [[ "$SAVE_MAP" == "true" ]]; then
        FASTLIO_ARGS+=(pcd_save_en:=true)
    fi
    "$PROJECT_ROOT/Scripts/fastlio/run_fastlio.sh" "${FASTLIO_ARGS[@]}" &
    FL_PID=$!
    sleep 5

    if ! kill -0 "$FL_PID" 2>/dev/null; then
        echo "[错误] FAST-LIO 基线阶段启动失败"
        exit 1
    fi

    echo "[A-2] 录制 /Odometry -> trajectory_baseline_offline.txt ..."
    rosrun ME5400 odom_to_tum_recorder.py \
        --output "$RESULT_DIR/trajectory_baseline_offline.txt" \
        _odom_topic:=/Odometry &
    REC_PID=$!
    sleep 2

    echo "[A-3] 启动离线 feeder (等待 FAST-LIO /Odometry 完成每帧)..."
    start_offline_feeder false false true "" "/Odometry" "0.20"

    echo "[A-4] 等待离线 baseline 阶段完成..."
    wait "$FEEDER_PID"
    FEEDER_PID=""

    sleep 3

    echo "[A-5] 停止 baseline recorder 与 FAST-LIO..."
    stop_pid "$REC_PID"
    REC_PID=""
    kill_matching_process "odom_to_tum_recorder.py"

    stop_pid "$FL_PID"
    FL_PID=""
    kill_matching_process "mapping_velodyne.launch"
    kill_matching_process "fastlio_mapping"
    kill_matching_process "laserMapping"
    kill_matching_process "fastlio_pose_bridge.py"

    archive_trajectory_if_new "$stage_start_ts" "$RESULT_DIR/trajectory_baseline_fastlio.txt"
    archive_map_if_new "$stage_start_ts" "$RESULT_DIR/scans_baseline_offline.pcd"
}

run_joint_offline() {
    local stage_start_ts
    stage_start_ts=$(date +%s)

    echo
    echo "========================================================="
    echo "   [离线阶段 C] JointOffline"
    echo "========================================================="

    echo "[C-1] 启动 PointPillars 节点..."
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
    PP_NODE_ARGS+=("_pointcloud_topic:=$JOINT_GATED_POINTCLOUD_TOPIC")
    BAG_FILE="" "$PROJECT_ROOT/Scripts/pointpillars/run_pointpillars_node.sh" _seq:="$SEQ_ID" "${PP_NODE_ARGS[@]}" &
    PP_PID=$!
    sleep 15

    echo "[C-2] 启动 MCTrack 在线节点..."
    "$PROJECT_ROOT/Scripts/mctrack/run_mctrack_online_node.sh" &
    MC_PID=$!
    sleep 3

    echo "[C-3] 启动 FAST-LIO (both, use_dynamic_weights=true)..."
    FASTLIO_ARGS=(
        both
        use_dynamic_weights:=true
        lidar_topic:=/kitti/velo/pointcloud
        imu_topic:=/kitti/oxts/imu
    )
    if [[ "$SAVE_MAP" == "true" ]]; then
        FASTLIO_ARGS+=(pcd_save_en:=true)
    fi
    "$PROJECT_ROOT/Scripts/fastlio/run_fastlio.sh" "${FASTLIO_ARGS[@]}" &
    FL_PID=$!
    sleep 5

    echo "[C-4] 启动联合后端..."
    "$PROJECT_ROOT/Scripts/joint_backend/run_joint_backend_ego.sh" --config "$JOINT_BACKEND_CONFIG_PATH" &
    JB_PID=$!
    sleep 3

    echo "[C-5] 录制 /joint_backend/odom -> trajectory_joint.txt ..."
    rosrun ME5400 odom_to_tum_recorder.py \
        --output "$RESULT_DIR/trajectory_joint.txt" \
        _odom_topic:=/joint_backend/odom &
    REC_PID=$!
    sleep 2

    echo "[C-6] 启动离线 feeder (FAST-LIO走原始流，PointPillars走gated点云；等待 detection / tracking)..."
    start_offline_feeder true true false "$JOINT_GATED_POINTCLOUD_TOPIC" "/joint_backend/odom" "0.06"

    echo "[C-7] 等待离线联合阶段完成..."
    wait "$FEEDER_PID"
    FEEDER_PID=""

    sleep 3

    echo "[C-8] 停止 recorder 与联合后端..."
    stop_pid "$REC_PID"
    REC_PID=""
    kill_matching_process "odom_to_tum_recorder.py"

    stop_pid "$JB_PID"
    JB_PID=""
    kill_matching_process "joint_backend_ego_node.py"

    echo "[C-9] 停止 FAST-LIO / MCTrack / PointPillars..."
    stop_pid "$FL_PID"
    FL_PID=""
    stop_pid "$MC_PID"
    MC_PID=""
    stop_pid "$PP_PID"
    PP_PID=""

    archive_trajectory_if_new "$stage_start_ts" "$RESULT_DIR/trajectory_weighted_offline.txt"
    archive_map_if_new "$stage_start_ts" "$RESULT_DIR/scans_weighted_offline.pcd"
}

run_evaluation() {
    local baseline_traj="$RESULT_DIR/trajectory_baseline_offline.txt"
    local weighted_traj="$RESULT_DIR/trajectory_weighted_offline.txt"
    local joint_traj="$RESULT_DIR/trajectory_joint.txt"

    # 检查基本轨迹是否存在
    if [[ ! -f "$baseline_traj" || ! -f "$joint_traj" ]]; then
        echo "[警告] 评估所需轨迹不完整，跳过自动评估"
        return
    fi

    echo
    echo "[步骤 4] 正在运行离线统一评估..."
    
    local eval_args=(
        --gt "$GT_PATH"
        --calib "$CALIB_PATH"
        --traj "Baseline(Offline,NoWeight):$baseline_traj"
    )

    # 如果存在权重优化的轨迹，则加入对比
    if [[ -f "$weighted_traj" ]]; then
        eval_args+=(--traj "FAST-LIO+MCTrack(Offline):$weighted_traj")
    fi

    # 加入联合后端轨迹
    eval_args+=(--traj "JointOffline:$joint_traj")

    python3 "$PROJECT_ROOT/Scripts/evaluation/evaluate_trajectories.py" \
        "${eval_args[@]}" \
        --output-dir "$RESULT_DIR" \
        --report-name "metrics.txt" \
        --report-json-name "metrics.json" \
        --report-title "离线轨迹评估报告" \
        --pair-plot-name "evaluation_result.png" \
        --overview-plot-name "evaluation_joint_backend.png" \
        --bar-plot-name "ablation_bar.png"
}

run_baseline_offline
cleanup_stage_processes
sleep 2

run_joint_offline
cleanup_stage_processes

run_evaluation

stop_pid "$PUB_PID"
PUB_PID=""

if [[ "$HEADLESS" == "true" ]]; then
    stop_pid "$RVIZ_PID"
    RVIZ_PID=""
fi

if [[ "$INTERNAL_ROSCORE" == "true" ]]; then
    stop_pid "$ROSCORE_PID"
    ROSCORE_PID=""
fi

SUCCESS=true

echo
echo "========================================================="
echo "   [离线双轨] 全部执行完成"
echo "   结果目录: Results/${SEQ_ID}_results/Offline/"
echo "========================================================="

if [[ "$HEADLESS" == "false" ]]; then
    echo "[INFO] 所有后台进程已停止，但保留 RViz 继续运行"
else
    echo "[INFO] 所有后台进程已停止"
fi
