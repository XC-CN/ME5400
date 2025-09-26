#!/bin/bash

# FAST-LIO2与KITTI数据运行脚本
# 此脚本同时启动FAST-LIO2 SLAM系统和KITTI数据播放

echo "设置ROS环境..."
source catkin_ws/devel/setup.bash

# 检查是否安装了终端模拟器
if command -v gnome-terminal &> /dev/null; then
    TERMINAL_CMD="gnome-terminal --"
elif command -v xterm &> /dev/null; then
    TERMINAL_CMD="xterm -e"
else
    echo "未找到支持的终端模拟器（gnome-terminal 或 xterm）"
    echo "请手动运行以下命令："
    echo "终端1: source catkin_ws/devel/setup.bash && roslaunch fast_lio mapping_velodyne.launch"
    echo "终端2: source catkin_ws/devel/setup.bash && rosbag play KITTI_Data/kitti_2011_09_26_drive_0019_sync.bag --clock"
    exit 1
fi

echo "启动FAST-LIO2映射节点..."
$TERMINAL_CMD bash -c "cd /home/xc/Projects/ME5400 && source catkin_ws/devel/setup.bash && roslaunch fast_lio mapping_velodyne.launch; exec bash"

echo "等待2秒后启动KITTI数据播放..."
sleep 2

echo "启动KITTI bag数据播放..."
$TERMINAL_CMD bash -c "cd /home/xc/Projects/ME5400 && source catkin_ws/devel/setup.bash && rosbag play KITTI_Data/kitti_2011_09_26_drive_0019_sync.bag --clock; exec bash"

echo "两个终端已启动："
echo "- 第一个终端运行FAST-LIO2"
echo "- 第二个终端播放KITTI数据"
echo "生成的点云地图将保存在 catkin_ws/src/fast_lio/PCD/scans.pcd"