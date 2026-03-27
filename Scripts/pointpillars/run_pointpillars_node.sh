#!/bin/bash

# KITTI PointPillars Bag ROS节点启动脚本

echo "启动KITTI PointPillars Bag ROS节点..."

# 脚本所在目录与仓库根目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
MMDET3D_ROOT="$PROJECT_ROOT/MMDET3D"

# 检查roscore是否运行
if ! pgrep -x "roscore" > /dev/null; then
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


# 启动ROS节点
echo "启动KITTI PointPillars Bag节点..."
python "$MMDET3D_ROOT/local/ros/nodes/kitti_pointpillars_bag_node.py" "$@" &
NODE_PID=$!

# 等待节点启动
sleep 5

# 播放bag文件
# 播放bag文件
if [ ! -z "$BAG_FILE" ]; then
    echo "播放bag文件: $BAG_FILE"
    rosbag play "$BAG_FILE" --clock --rate=0.5  # 0.5倍速播放
fi

# 等待处理完成
wait $NODE_PID

echo "Bag处理完成"
