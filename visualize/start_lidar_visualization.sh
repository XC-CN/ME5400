#!/bin/bash

echo "=== nuScenes LiDAR可视化启动脚本 ==="

# 检查ROS核心是否运行
if ! rostopic list > /dev/null 2>&1; then
    echo "启动ROS核心..."
    roscore &
    sleep 3
fi

# 激活conda环境
echo "激活conda环境..."
source /home/xc/miniconda3/bin/activate MCTrack

# 检查数据集路径
DATAROOT="/home/xc/Projects/ME5400/ME5400/data/nuScenes"
if [ ! -d "$DATAROOT" ]; then
    echo "错误: 数据集路径不存在: $DATAROOT"
    exit 1
fi

echo "数据集路径: $DATAROOT"
echo ""
echo "=== 自动启动Rviz和LiDAR可视化器 ==="
echo ""

# 禁用ROS 1生命周期警告
export DISABLE_ROS1_EOL_WARNINGS=1

# 启动Rviz（使用配置文件，全屏显示）
echo "启动Rviz（使用预配置的显示设置）..."
echo "Rviz窗口将全屏显示，左侧面板已优化..."

# 启动Rviz
rviz -d visualize/nuscenes_lidar.rviz &
RVIZ_PID=$!

# 等待Rviz启动
sleep 2
echo "Rviz已启动，界面布局已优化"

# 自动处理ROS警告弹窗
echo "处理ROS警告弹窗..."
sleep 1
# 查找并关闭ROS警告对话框
ROS_WARNING_ID=$(wmctrl -l | grep -i "ROS 1 End-of-Life" | awk '{print $1}' | head -1)
if [ ! -z "$ROS_WARNING_ID" ]; then
    echo "发现ROS警告弹窗，自动关闭..."
    wmctrl -i -c $ROS_WARNING_ID
    sleep 0.5
fi

# 等待Rviz启动完成
echo "等待Rviz启动完成..."
sleep 2

echo ""
echo "=== Rviz已自动配置 ==="
echo "✓ 已添加PointCloud2显示"
echo "✓ 话题设置为: /nuscenes/lidar"
echo "✓ 坐标系设置为: lidar"
echo "✓ 固定坐标系设置为: map"
echo "✓ 点云颜色基于强度值"
echo ""

# 启动LiDAR可视化器
echo "启动LiDAR可视化器..."
echo "按Ctrl+C停止播放"
echo ""

python3 visualize/nuscenes_rviz_visualizer.py "$DATAROOT"

# 清理：停止Rviz
echo ""
echo "停止Rviz..."
kill $RVIZ_PID 2>/dev/null
