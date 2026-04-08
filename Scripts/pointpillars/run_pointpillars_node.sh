#!/bin/bash
set -e

# KITTI PointPillars Bag ROS节点启动脚本

echo "启动KITTI PointPillars Bag ROS节点..."

# 脚本所在目录与仓库根目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
MMDET3D_ROOT="$PROJECT_ROOT/MMDET3D"

source "$PROJECT_ROOT/Scripts/utils/setup_ros_env.sh"

SEQ_PARAM=""
DATASET_ROOT_PARAM=""
NODE_ARGS=()

for arg in "$@"; do
    case "$arg" in
        _seq:=*)
            SEQ_PARAM="${arg#_seq:=}"
            ;;
        _dataset_root:=*)
            DATASET_ROOT_PARAM="${arg#_dataset_root:=}"
            ;;
        *)
            NODE_ARGS+=("$arg")
            ;;
    esac
done

SEQ_PARAM_DECIMAL=""
if [[ -n "$SEQ_PARAM" ]]; then
    SEQ_PARAM_DECIMAL=$((10#$SEQ_PARAM))
fi

ORIG_SEQ_SET=false
ORIG_SEQ_VALUE=""
ORIG_DATASET_SET=false
ORIG_DATASET_VALUE=""

cleanup() {
    if [[ -n "${NODE_PID:-}" ]]; then
        kill "$NODE_PID" 2>/dev/null || true
        wait "$NODE_PID" 2>/dev/null || true
    fi

    if [[ "$ORIG_SEQ_SET" == "true" ]]; then
        rosparam set /me5400_seq "$ORIG_SEQ_VALUE" >/dev/null 2>&1 || true
    elif [[ -n "$SEQ_PARAM_DECIMAL" ]]; then
        rosparam delete /me5400_seq >/dev/null 2>&1 || true
    fi

    if [[ "$ORIG_DATASET_SET" == "true" ]]; then
        rosparam set /me5400_dataset_root "$ORIG_DATASET_VALUE" >/dev/null 2>&1 || true
    elif [[ -n "$DATASET_ROOT_PARAM" ]]; then
        rosparam delete /me5400_dataset_root >/dev/null 2>&1 || true
    fi
}

trap cleanup EXIT INT TERM

# 检查roscore是否运行
if ! rostopic list > /dev/null 2>&1; then
    echo "启动roscore..."
    roscore &
    sleep 3
fi

# 激活conda环境
echo "激活ME5400环境..."
source ~/miniconda3/etc/profile.d/conda.sh
conda activate ME5400

# 设置ROS环境变量
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$PROJECT_ROOT
export PYTHONPATH=$PYTHONPATH:$PROJECT_ROOT

if ORIG_SEQ_VALUE=$(rosparam get /me5400_seq 2>/dev/null); then
    ORIG_SEQ_SET=true
fi
if ORIG_DATASET_VALUE=$(rosparam get /me5400_dataset_root 2>/dev/null); then
    ORIG_DATASET_SET=true
fi

if [[ -n "$SEQ_PARAM_DECIMAL" ]]; then
    rosparam set /me5400_seq "$SEQ_PARAM_DECIMAL"
fi
if [[ -n "$DATASET_ROOT_PARAM" ]]; then
    rosparam set /me5400_dataset_root "$DATASET_ROOT_PARAM"
fi

# 启动ROS节点
echo "启动KITTI PointPillars Bag节点..."
python "$MMDET3D_ROOT/local/ros/nodes/kitti_pointpillars_bag_node.py" "${NODE_ARGS[@]}" &
NODE_PID=$!

# 等待节点启动
sleep 5

# 播放bag文件
if [[ -n "$BAG_FILE" ]]; then
    echo "播放bag文件: $BAG_FILE"
    rosbag play "$BAG_FILE" --clock --rate=0.5
fi

# 等待处理完成
wait "$NODE_PID"

echo "Bag处理完成"
