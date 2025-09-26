# ME5400: Advanced Robotics and Autonomous Systems

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![ROS Version](https://img.shields.io/badge/ROS-Noetic-blue.svg)](http://wiki.ros.org/noetic)
[![Python Version](https://img.shields.io/badge/Python-3.8+-green.svg)](https://www.python.org/)

这是ME5400高级机器人与自主系统课程项目，集成了**多目标跟踪**和**激光雷达SLAM**技术，用于自动驾驶场景下的感知和建图。

## 📋 项目概述

本项目包含两个主要组件：

1. **MCTrack**: 基于卡尔曼滤波的多目标跟踪系统，支持KITTI、nuScenes、Waymo等数据集
2. **FAST-LIO2**: 修改版的快速激光雷达-惯性里程计，适配Velodyne激光雷达和KITTI数据集

## 🚀 主要功能

### MCTrack 多目标跟踪

- 🎯 支持多种数据集（KITTI、nuScenes、Waymo）
- 🔄 基于卡尔曼滤波的状态估计
- 📊 完整的评估框架（HOTA、CLEAR等指标）
- 🎨 可视化工具和结果分析

### FAST-LIO2 激光雷达SLAM

- ⚡ 高效的激光雷达-惯性里程计
- 🗺️ 实时3D建图和定位
- 🔧 支持Velodyne激光雷达
- 📦 KITTI数据集适配

## 📁 项目结构

```
ME5400/
├── MCTrack/                 # 多目标跟踪系统
│   ├── tracker/             # 跟踪算法核心
│   ├── evaluation/          # 评估工具
│   ├── config/              # 配置文件
│   └── results/             # 实验结果
├── catkin_ws/               # ROS工作空间
│   └── src/fast_lio/        # FAST-LIO2源码
├── KITTI_Data/              # KITTI数据集文件夹（gitignore排除）
│   ├── kitti_to_rosbag.py   # 数据转换工具
│   ├── *.bag                # ROS bag文件
│   ├── *.zip                # 原始数据包
│   └── 2011_09_26/          # 解压后数据
└── README.md                # 项目说明
```

## 🛠️ 环境要求

### 系统要求

- Ubuntu 20.04 LTS
- ROS Noetic
- Python 3.8+

### 依赖库

```bash
# ROS依赖
sudo apt install ros-noetic-pcl-ros ros-noetic-eigen-conversions

# Python依赖
pip install numpy opencv-python matplotlib
```

## ⚙️ 安装步骤

### 1. 克隆仓库

```bash
git clone https://github.com/XC-CN/ME5400.git
cd ME5400
```

### 2. 编译FAST-LIO2

```bash
cd catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

### 3. 安装MCTrack依赖

```bash
cd MCTrack
pip install -r requirements.txt
```

## 🎯 使用指南

### FAST-LIO2 建图

#### 1. 数据准备

将KITTI数据转换为ROS bag格式：

```bash
cd KITTI_Data
python3 kitti_to_rosbag.py
```

#### 2. 运行FAST-LIO2

使用提供的脚本快速启动：

```bash
# 运行FAST-LIO2和KITTI数据
./run_fastlio2_kitti.sh
```

或者手动执行：

```bash
# 启动FAST-LIO2
cd catkin_ws
source devel/setup.bash
roslaunch fast_lio mapping_velodyne.launch

# 播放数据（新终端）
rosbag play KITTI_Data/kitti_2011_09_26_drive_0019_sync.bag --clock
```

#### 3. 结果查看

生成的点云地图保存在：

```
catkin_ws/src/fast_lio/PCD/scans.pcd
```

可使用PCL工具查看：

```bash
pcl_viewer catkin_ws/src/fast_lio/PCD/scans.pcd
```

### MCTrack 多目标跟踪

#### 1. 配置数据集

编辑对应的配置文件：

```bash
# KITTI数据集
vim ME5400_MCTrack/config/kitti.yaml

# nuScenes数据集  
vim ME5400_MCTrack/config/nuscenes.yaml
```

#### 2. 运行跟踪

```bash
cd MCTrack
python main.py --config config/kitti.yaml
```

#### 3. 评估结果

```bash
# 运行评估
python evaluation/eval_motion.py

# 查看结果
ls results/
```

## 📊 实验结果

### FAST-LIO2 建图效果

- **处理数据**: KITTI 2011_09_26_drive_0019_sync (481帧)
- **生成点云**: 14,508,582个点
- **地图文件**: 464MB PCD格式

### MCTrack 跟踪性能

- **KITTI数据集**: 支持车辆跟踪评估
- **评估指标**: HOTA、CLEAR、Identity等
- **可视化**: 提供轨迹和性能图表

## 🔧 配置说明

### FAST-LIO2 配置

主要配置文件：`catkin_ws/src/fast_lio/config/velodyne.yaml`

```yaml
common:
  lid_topic: "/kitti/velo/pointcloud"  # 激光雷达话题
  imu_topic: "/kitti/oxts/imu"         # IMU话题

preprocess:
  lidar_type: 2                        # Velodyne激光雷达
  scan_line: 64                        # 扫描线数
  scan_rate: 10                        # 扫描频率

mapping:
  extrinsic_T: [0, 0, 0.28]           # 外参平移
  extrinsic_R: [1, 0, 0,              # 外参旋转
                0, 1, 0,
                0, 0, 1]
```

### MCTrack 配置

主要配置文件：`ME5400_MCTrack/config/kitti.yaml`

## 🤝 贡献

欢迎提交Issue和Pull Request！

## 📄 许可证

本项目采用MIT许可证 - 查看 [LICENSE](LICENSE) 文件了解详情。

## 📚 参考文献

1. **FAST-LIO2**: [Fast Direct LiDAR-Inertial Odometry](https://github.com/hku-mars/FAST_LIO)
2. **MCTrack**: Multi-Object Tracking with Motion Compensation
3. **KITTI Dataset**: [Vision meets Robotics](http://www.cvlibs.net/datasets/kitti/)

## 👥 作者

- **XC-CN** - 项目维护者

## 🔗 相关链接

- [项目仓库](https://github.com/XC-CN/ME5400.git)
- [FAST-LIO原版](https://github.com/hku-mars/FAST_LIO)
- [KITTI数据集](http://www.cvlibs.net/datasets/kitti/)

---

⭐ 如果这个项目对你有帮助，请给个Star！
