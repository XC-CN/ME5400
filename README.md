# ME5400: Advanced Robotics and Autonomous Systems

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![ROS Version](https://img.shields.io/badge/ROS-Noetic-blue.svg)](http://wiki.ros.org/noetic)
[![Python Version](https://img.shields.io/badge/Python-3.8+-green.svg)](https://www.python.org/)

这是ME5400高级机器人与自主系统课程项目，聚焦于**FAST-LIO 里程计与 MCTrack 多目标跟踪的融合管线**，用于自动驾驶场景下将实时点云/IMU 位姿与三维目标检测数据统一处理并可视化。

## 📋 项目概述

本项目包含两个主要组件：

1. **FAST-LIO2**：提供点云+IMU 的实时里程计估计
2. **MCTrack**：在 FAST-LIO 位姿下对检测目标进行 3D 多目标跟踪，并生成包围盒、轨迹与朝向信息

当前检测输入直接来自 KITTI Tracking 数据集提供的检测结果；后续可替换为任意 3D/多模态目标检测器的实时输出。

## 🚀 主要功能

- 🔗 **FAST-LIO × MCTrack 融合管线**：将点云/IMU 里程计与检测目标对齐，实时输出包围盒、轨迹与朝向箭头。
- 🎯 **多数据集兼容的跟踪模块**：MCTrack 支持 KITTI / nuScenes / Waymo；当前默认使用 KITTI Tracking 的检测结果。
- ⚡ **FAST-LIO 实时里程计**：针对 Velodyne 激光雷达的高效激光雷达-惯性里程计，可直接处理实时点云。
- 🎨 **统一可视化脚本**：提供在线融合脚本，默认的检测输入来自数据集（未来可改接任意目标检测算法）。

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

### 在线 ROS 联动：KITTI Tracking 点云 → FAST-LIO → MCTrack → RViz

> 当前检测由 KITTI Tracking 数据集提供的检测文件生成并发布；后续可替换为任意目标检测算法的 ROS 输出。

运行前请准备好 KITTI Tracking 数据目录：

- `tracking/training/`：需包含 `calib/`、`oxts/`、`velodyne/` 等子目录。
- `tracking/det_tracking_lsvm/`：保持检测结果组织为 `training/det_02/<seq>.txt` 结构，或替换成你自己的检测输出。
- 将点云+IMU 数据转换为 rosbag 并放在 `tracking/rosbags/`，示例脚本见下。

> 当前版本仅通过播放 KITTI Tracking 序列的 rosbag 来驱动联动流程，暂未直接接入实时传感器。

可使用提供的工具脚本从 KITTI Tracking 原始数据生成所需 rosbag（默认同时写入检测结果）：

```bash
./Scripts/kitti_tracking_to_rosbag.py
```

该 bag 由 KITTI Tracking `training/` 序列（Velodyne 点云 + OXTS IMU）转换而来，包含 `/kitti/velo/pointcloud`、`/kitti/oxts/imu` 与 `/kitti/detections` 等话题。默认使用序列 **0019**（1059 帧，约 106 s）。如需仅含点云/IMU，可去掉 `--include_detections` 并使用不同输出文件名（例如 `seq_0019.bag`）。

1. **启动 roscore（终端 A）**
   ```bash
   roscore
   ```

2. **加载 ROS 环境（终端 B 及后续每个终端）**
   ```bash
   cd ~/Projects/ME5400
   source Scripts/setup_ros_env.sh
   ```

3. **【首次或代码更新后】编译工作空间**
   ```bash
   ./Scripts/build_catkin_ws.sh
   ```

4. **【可选】生成或更新默认 rosbag**
   ```bash
   ./Scripts/kitti_tracking_to_rosbag.py
   ```
   输出保存到 `tracking/rosbags/seq_0019_with_det.bag`，包含点云、IMU 与检测话题。

5. **播放默认 rosbag（终端 C，保持运行）**
   ```bash
   ./Scripts/play_kitti_rosbag.sh
   ```

6. **打开 RViz（终端 D，与 rosbag 同时运行）**
   ```bash
   ./Scripts/run_rviz.sh
   ```
   使用 `kitti_simple.rviz` 配置实时查看点云、检测和 MCTrack 目标。

7. **启动 FAST-LIO 映射（终端 E）**
   ```bash
   ./Scripts/run_fastlio_mapping.sh
   ```

8. **启动 FAST-LIO → Pose 桥接（终端 F）**
   ```bash
   ./Scripts/run_fastlio_pose_bridge.sh
   ```

9. **启动 MCTrack 在线节点（终端 G）**
   ```bash
   ./Scripts/run_mctrack_online_node.sh
   ```
   节点订阅 `/mctrack/lidar_pose` 与 `/kitti/detections`，实时发布 `/mctrack/markers`。

**【可选】一键启动全部节点**
    ```bash
    ./Scripts/run_mctrack_online.sh
    ```
    脚本会自动处理 roscore、rosbag 播放、检测发布、FAST-LIO、姿态桥、MCTrack 及 RViz，使用脚本默认路径。

### 无窗口环境下验证 RViz（自动截图）

1. 确保 ROS 节点已运行并有数据流。
2. 执行：
   ```bash
   python Scripts/rviz_headless_check.py \
     catkin_ws/src/fast_lio/rviz_cfg/kitti_simple.rviz \
     --screenshot rviz_headless.png
   ```
3. 脚本会：
   - 使用 `xvfb-run` 启动 RViz；
   - 检查日志是否有 `[ERROR]`、段错误；
   - 自动解析 RViz 配置里的订阅主题并与 `rostopic list` 对比；
   - 调用 `rviz_screenshot` 生成截图；
   - 输出 JSON 报告，如：
     ```json
     {
       "rviz_process": "running",
       "log_status": "ok",
       "topics_verified": ["/kitti/velo/pointcloud", "/tf"],
       "topics_missing": [],
       "screenshot": {"path": "rviz_headless.png", "exists": true, "size_kb": 123.4, "success": true},
       "conclusion": "rviz可视化正常"
     }
     ```
4. 如出现错误，会给出失败原因并清理临时文件。

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
