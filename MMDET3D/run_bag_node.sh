#!/bin/bash

# KITTI PointPillars Bag ROS节点启动脚本

echo "启动KITTI PointPillars Bag ROS节点..."

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
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export PYTHONPATH=$PYTHONPATH:$(pwd)

# 检查bag文件是否存在
BAG_FILE="data/kitti/seq_0019_with_det.bag"
if [ ! -f "$BAG_FILE" ]; then
    echo "错误: bag文件不存在: $BAG_FILE"
    exit 1
fi

echo "Bag文件: $BAG_FILE"

# 启动ROS节点
echo "启动KITTI PointPillars Bag节点..."
python kitti_pointpillars_bag_node.py &
NODE_PID=$!

# 等待节点启动
sleep 5

# 播放bag文件
echo "播放bag文件..."
rosbag play $BAG_FILE --clock --rate=0.5  # 0.5倍速播放

# 等待处理完成
wait $NODE_PID

echo "Bag处理完成"
