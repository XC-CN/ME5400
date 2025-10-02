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

#### 数据目录说明

- `MCTrack/data/`：正式使用的数据目录，包含按照 BaseVersion 规范整理好的 `datasets/`（原始标注、pose、calib 等）与 `base_version/`（JSON 格式的检测结果）。
- `MCTrack/data_download_temp/`：官方脚本下载压缩包时的临时缓存，若使用项目自带脚本直接整理数据可忽略；若存在，可安全删除以节省磁盘空间。

#### 1. 准备 KITTI Tracking 数据

```bash
# 进入项目根目录
cd /home/xc/Projects/ME5400

# 1) 生成 pose 文本（依赖 pykitti，已写入 conda 环境）
conda run -n MCTrack python Scripts/generate_kitti_pose.py

# 2) 将 label_02 转换为检测输入（仅保留 Car 类）
conda run -n MCTrack python Scripts/convert_gt_to_detector.py

# 3) 生成 BaseVersion JSON（输出至 MCTrack/data/base_version/kitti/gt/val.json）
cd MCTrack
conda run -n MCTrack python preprocess/convert_kitti.py \
  --raw_data_path data/kitti/datasets/ \
  --dets_path data/kitti/detectors/ \
  --save_path data/base_version/kitti/ \
  --detector gt --split val
```

完成后，确保 `config/kitti.yaml` 中的 `DETECTOR` 设为 `gt`（本仓库已配置）。

#### 2. 运行 MCTrack 跟踪 + 评估

```bash
cd /home/xc/Projects/ME5400/MCTrack
conda run -n MCTrack python main.py --dataset kitti -e -p 1
```

执行成功后，结果会写入 `MCTrack/results/kitti/<时间戳>/gt/val/`，其中包含逐序列的 `data/*.txt` 跟踪输出、`car_summary.txt` 指标及图表。

#### 3. RViz 可视化（推荐一键脚本）

```bash
# 默认播放 kitti_2011_09_26_drive_0019_sync.bag，并展示 0000 序列的跟踪结果
./Scripts/run_mctrack_viz.sh

# 如需指定参数（示例：使用其它结果文件 / bag）
./Scripts/run_mctrack_viz.sh \
  --bag KITTI_Data/your_sequence.bag \
  --result MCTrack/results/kitti/20251002_174627/gt/val/data/0001.txt \
  --calib MCTrack/data/kitti/datasets/training/calib/0001.txt \
  --frame velo_link --rate 10

# 仅查看将要执行的命令
./Scripts/run_mctrack_viz.sh --print
```

脚本会自动检测 `gnome-terminal`/`xterm`，依次启动 `roscore`、`rosbag play`、`mctrack_marker_publisher.py`、`rviz`。如环境不支持自动开新终端，脚本会提示需手动执行的命令。

#### 4. 使用 FAST-LIO 里程计驱动 MCTrack（离线 JSON）

1. **重新编译 ROS 包（添加了新的记录脚本）**
   ```bash
   cd /home/xc/Projects/ME5400/catkin_ws
   catkin_make
   source devel/setup.bash
   ```

2. **启动 FAST-LIO 并播放 bag（和之前步骤一致）**
   ```bash
   # 终端1：FAST-LIO
   roslaunch fast_lio mapping_velodyne.launch

   # 终端2：播放含 Velodyne+IMU 的 KITTI bag
   rosbag play ../KITTI_Data/kitti_2011_09_26_drive_0019_sync.bag --clock --pause
   ```
   （建议先暂停 `rosbag`，等下启动录制后再继续）

3. **在新的终端录制 FAST-LIO 位姿，同步点云帧**
   ```bash
   rosrun kitti_tracklets_viz fastlio_pose_recorder.py \
     --output /home/xc/Projects/ME5400/MCTrack/data/kitti/datasets/training/pose_fastlio \
     --seq 0000 \
     _odom_topic:=/aft_mapped_to_init \
     _point_topic:=/kitti/velo/pointcloud
   ```
   之后在播放 rosbag 的终端按空格继续，让 bag 跑完整个序列；录制脚本会在 `pose_fastlio/0000.txt` 中写入每帧 3×4 位姿矩阵。

4. **基于 FAST-LIO 位姿生成 BaseVersion JSON**（仅处理序列 0000，避免覆盖原结果）
   ```bash
   cd /home/xc/Projects/ME5400/MCTrack
   conda run -n MCTrack python preprocess/convert_kitti.py \
     --raw_data_path data/kitti/datasets/ \
     --dets_path data/kitti/detectors/ \
     --save_path data/base_version_fastlio/kitti/ \
     --detector gt \
     --split val \
     --pose_root pose_fastlio \
     --seqs 0000
   ```
   生成的 JSON 位于 `data/base_version_fastlio/kitti/gt/val.json`，其中 `global2lidar` 采用 FAST-LIO 里程计。

5. **使用专门配置运行 MCTrack**（只追踪序列 0000）
   ```bash
   cd /home/xc/Projects/ME5400/MCTrack
   conda run -n MCTrack python main.py --dataset kitti --eval -p 1 \
     --config config/kitti_fastlio.yaml
   ```

6. **可视化**：运行 `./Scripts/run_mctrack_viz.sh --result MCTrack/results/kitti/<时间戳>/gt/val/data/0000.txt`，即可在 RViz 中查看使用 FAST-LIO 里程计的跟踪结果。

### 在线 ROS 联动：rosbag → FAST-LIO → MCTrack → RViz

> 适合把 rosbag（或实时激光雷达）流式传入 FAST-LIO，同时发布检测结果，并让 MCTrack 在线输出轨迹/包围盒。

1. **重新编译消息与节点**
   ```bash
   cd /home/xc/Projects/ME5400/catkin_ws
   catkin_make
   source devel/setup.bash
   ```

2. **准备检测发布器**（若使用 KITTI LSVm 检测）
   ```bash
   rosrun kitti_tracklets_viz kitti_detection_publisher.py \
     --dataset_root /home/xc/Projects/ME5400/tracking \
     --detector_root /home/xc/Projects/ME5400/tracking/det_tracking_lsvm \
     --seq 0 --rate 10
   ```
   该节点读取 `det_tracking_lsvm/<seq>.txt`，输出话题 `/kitti/detections`。

3. **FAST-LIO 与里程计桥接**
   - 启动 FAST-LIO （`roslaunch fast_lio mapping_velodyne.launch`），确保其输出 `/aft_mapped_to_init`。
   - 运行桥接节点：
     ```bash
     rosrun kitti_tracklets_viz fastlio_pose_bridge.py \
       _odom_topic:=/aft_mapped_to_init \
       _pose_topic:=/mctrack/lidar_pose
     ```

4. **MCTrack 在线跟踪节点**
   ```bash
   rosrun kitti_tracklets_viz mctrack_online_node.py \
     --dataset_root /home/xc/Projects/ME5400/tracking \
     --seq 0 \
     --config /home/xc/Projects/ME5400/MCTrack/config/kitti_fastlio.yaml
   ```
   节点订阅 `/mctrack/lidar_pose` 与 `/kitti/detections`，实时输出 `/mctrack/markers`（轨迹、OBB、朝向箭头）。

5. **批量启动脚本**
   ```bash
   ./Scripts/run_mctrack_online.sh \
     --bag KITTI_Data/kitti_2011_09_26_drive_0019_sync.bag \
     --dataset /home/xc/Projects/ME5400/tracking \
     --detector_root /home/xc/Projects/ME5400/tracking/det_tracking_lsvm \
     --seq 0
   ```
   该脚本自动启动 roscore、rosbag（点云+IMU）、检测发布器、FAST-LIO 里程计桥、MCTrack 在线节点以及 RViz（可加 `--norviz` 关闭）。

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
