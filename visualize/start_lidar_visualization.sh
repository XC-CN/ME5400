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

# 启动Rviz（使用配置文件，自动填满左半边屏幕）
echo "启动Rviz（使用预配置的显示设置）..."
echo "Rviz窗口将自动填满左半边屏幕..."

# 获取屏幕尺寸并计算左半边窗口位置
SCREEN_WIDTH=$(xrandr --current | grep '*' | head -1 | awk '{print $1}' | cut -d'x' -f1)
SCREEN_HEIGHT=$(xrandr --current | grep '*' | head -1 | awk '{print $1}' | cut -d'x' -f2)
HALF_WIDTH=$((SCREEN_WIDTH / 2))

# 启动Rviz并设置窗口位置
rviz -d visualize/nuscenes_lidar.rviz &
RVIZ_PID=$!

# 等待Rviz启动后调整窗口位置
sleep 2
echo "调整Rviz窗口位置到左半边屏幕..."

# 查找并调整Rviz窗口位置
RVIZ_WINDOW_ID=$(wmctrl -l | grep -i "rviz" | awk '{print $1}' | head -1)
if [ ! -z "$RVIZ_WINDOW_ID" ]; then
    wmctrl -i -r $RVIZ_WINDOW_ID -e 0,0,0,$HALF_WIDTH,$SCREEN_HEIGHT
    echo "Rviz窗口已调整到左半边屏幕"
else
    echo "未找到Rviz窗口，跳过位置调整"
fi

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
