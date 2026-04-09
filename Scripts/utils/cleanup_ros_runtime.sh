#!/bin/bash
set -euo pipefail

SCRIPT_NAME=$(basename "$0")

log() {
    echo "[$SCRIPT_NAME] $*"
}

terminate_matching_processes() {
    local signal="$1"
    shift
    local description pattern

    while [[ $# -gt 0 ]]; do
        description="$1"
        pattern="$2"
        shift 2

        mapfile -t pids < <(pgrep -f "$pattern" 2>/dev/null || true)
        if [[ ${#pids[@]} -eq 0 ]]; then
            continue
        fi

        log "发送 SIG${signal} 给 ${description}: ${pids[*]}"
        pkill "-${signal}" -f "$pattern" 2>/dev/null || true
    done
}

log "正在清理残留 ROS 运行时进程..."

terminate_matching_processes TERM \
    "RViz" "rviz( |$)" \
    "GT path publisher" "publish_gt_path.py" \
    "rosbag play" "rosbag play" \
    "Offline bag feeder" "offline_bag_feeder.py" \
    "FAST-LIO roslaunch" "mapping_velodyne.launch" \
    "FAST-LIO pose bridge" "fastlio_pose_bridge.py" \
    "PointPillars bag node" "kitti_pointpillars_bag_node.py" \
    "MCTrack online node" "mctrack_online_node.py" \
    "Joint backend ego node" "joint_backend_ego_node.py" \
    "Odometry TUM recorder" "odom_to_tum_recorder.py"

sleep 1

terminate_matching_processes KILL \
    "RViz" "rviz( |$)" \
    "GT path publisher" "publish_gt_path.py" \
    "rosbag play" "rosbag play" \
    "Offline bag feeder" "offline_bag_feeder.py" \
    "FAST-LIO roslaunch" "mapping_velodyne.launch" \
    "FAST-LIO pose bridge" "fastlio_pose_bridge.py" \
    "PointPillars bag node" "kitti_pointpillars_bag_node.py" \
    "MCTrack online node" "mctrack_online_node.py" \
    "Joint backend ego node" "joint_backend_ego_node.py" \
    "Odometry TUM recorder" "odom_to_tum_recorder.py"

if rostopic list >/dev/null 2>&1; then
    cleanup_output=$(printf 'y\n' | rosnode cleanup 2>&1 || true)
    if grep -q "Unregistering" <<<"$cleanup_output"; then
        log "已清理 ROS master 中的失联节点注册。"
    else
        log "ROS master 中没有需要清理的失联节点。"
    fi
else
    log "未检测到可用 roscore，跳过 rosnode cleanup。"
fi
