# ME5400: Advanced Robotics and Autonomous Systems

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![ROS Version](https://img.shields.io/badge/ROS-Noetic-blue.svg)](http://wiki.ros.org/noetic)
[![Python Version](https://img.shields.io/badge/Python-3.8+-green.svg)](https://www.python.org/)

这是ME5400高级机器人与自主系统课程项目，聚焦于**FAST-LIO 里程计与 MCTrack 多目标跟踪的融合管线**，用于自动驾驶场景下将实时点云/IMU 位姿与三维目标检测数据统一处理并可视化。

## 📋 项目概述

本项目包含两个主要组件：

1. **FAST-LIO2**：提供点云+IMU 的实时里程计估计
2. **MCTrack**：在 FAST-LIO 位姿下对检测目标进行 3D 多目标跟踪，并生成包围盒、轨迹与朝向信息

项目内置 MMDetection3D PointPillars 推理节点作为统一检测器；如需切换，可替换为其他 3D/多模态目标检测算法的实时输出。

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

### PointPillars 快速环境配置（ME5400）

- **数据集路径**：`MMDET3D_KITTI/data/kitti/`（rosbag 示例位于 `MMDET3D_KITTI/data/kitti/seq_0019_with_det.bag`）
- **Conda 环境**：`ME5400`，Python 3.10.x
- **关键依赖版本**：CUDA Toolkit 12.8（含 NVCC）、PyTorch 2.10.0.dev+cu128（nightly）、MMCV 2.1.0（CUDA 12.8 源码编译版）、MMEngine 0.10.7、MMDetection 3.2.0、MMDetection3D 1.4.0
- **创建与激活环境**

  ```bash
  conda create -n ME5400 python=3.10
  conda activate ME5400
  ```
- **安装 CUDA 12.8（需管理员或本地 Toolkit）**

  1. 安装 NVIDIA 550+ 驱动并下载 [CUDA Toolkit 12.8](https://developer.nvidia.com/cuda-12-8-0-download-archive)。
  2. 仅选择 `cuda-toolkit` 组件（驱动已安装可跳过），安装路径建议 `/usr/local/cuda-12.8`。
  3. 在 `~/.bashrc` 增加：

     ```bash
     export CUDA_HOME=/usr/local/cuda-12.8
     export PATH=$CUDA_HOME/bin:$PATH
     export LD_LIBRARY_PATH=$CUDA_HOME/lib64:$LD_LIBRARY_PATH
     ```

     重新 `source ~/.bashrc`。
- **安装 PyTorch Nightly（CUDA 12.8）**

  ```bash
  pip install --upgrade pip
  pip install --pre torch torchvision torchaudio --index-url https://download.pytorch.org/whl/nightly/cu128
  ```
- **安装 OpenMMLab 组件（CUDA 12.8 编译）**

  ```bash
  pip install openmim
  mim install mmengine==0.10.7
  # 需要 NVCC，确保 CUDA_HOME 指向 Toolkit 12.8
  export CUDA_HOME=${CUDA_HOME:-/usr/local/cuda-12.8}
  MMCV_WITH_OPS=1 FORCE_CUDA=1 MAX_JOBS=4 mim install mmcv==2.1.0
  pip install mmdet==3.2.0 mmdet3d==1.4.0
  ```

  > 构建 `mmcv` 时会自动编译 CUDA/C++ 算子，耗时 5~10 分钟，请耐心等待；若内存较小可追加 `MAX_JOBS=4`。
  > 注意：目前 `mmdet3d==1.4.0` 对 `mmcv` 的版本上限为 `< 2.2.0`，请勿安装更高版本。
- **常用依赖**

  ```bash
  pip install open3d==0.19.0 opencv-python==4.12.0.88 numpy==2.1.2 matplotlib==3.10.6 scipy==1.15.3 \
              scikit-learn==1.7.2 pandas==2.3.3 pillow==11.3.0 pyyaml==6.0.3 tqdm terminaltables==3.1.10 \
              shapely==1.8.5.post1 pyquaternion==0.9.9 trimesh==4.8.3 plyfile==1.1.2 imageio==2.37.0 \
              fire==0.7.1 tensorboard==2.20.0 protobuf==6.32.1
  pip install rospkg==1.6.0 catkin-pkg==1.1.0
  ```
- **安装验证**

  ```bash
  python -c "import torch; print('PyTorch版本:', torch.__version__); print('CUDA可用:', torch.cuda.is_available()); print('编译支持架构:', torch.cuda.get_arch_list())"
  python -c "import mmcv; print('MMCV版本:', mmcv.__version__)"
  python -c "import mmdet3d; print('MMDetection3D版本:', mmdet3d.__version__)"
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

### 在线 ROS 联动：KITTI Tracking 点云 → PointPillars → FAST-LIO → MCTrack → RViz

> 当前检测由 MMDetection3D PointPillars 推理节点发布至 `/detection/bboxes_3d` 与 `/detection/kitti_tracking`；后续如需替换，可接入其他检测器但需保持话题接口一致。

运行前请准备好 KITTI Tracking 数据目录（用于工况播放与 MCTrack 标定信息）：

- `tracking/training/`：需包含 `calib/`、`oxts/`、`velodyne/` 等子目录。
- `tracking/det_tracking_lsvm/`：仅在评估历史检测时使用；实时检测场景可忽略。
- 将点云+IMU 数据转换为 rosbag 并放在 `tracking/rosbags/`，或者直接使用仓库提供的示例 bag。

> 当前流程仍通过播放 KITTI Tracking 序列的 rosbag 来驱动，暂未直接接入实时传感器。

可使用提供的工具脚本从 KITTI Tracking 原始数据生成所需 rosbag（默认只需点云与 IMU）：

```bash
./Scripts/kitti_tracking_to_rosbag.py --seq 19 --include_detections False
```

也可以直接使用 `MMDET3D_KITTI/data/kitti/seq_0019_with_det.bag` 或 `tracking/rosbags/seq_0019_with_det.bag` 作为示例数据。

1. **启动 roscore（终端 A）**

   ```bash
   roscore
   ```
2. **【首次或代码更新后】编译工作空间**

   ```bash
   ./Scripts/build_catkin_ws.sh
   ```
3. **【可选】生成或更新默认 rosbag**

   ```bash
   ./Scripts/kitti_tracking_to_rosbag.py --seq 19
   ```

4. **启动 PointPillars 检测节点（终端 C）**

   ```bash
   conda activate ME5400
   python MMDET3D_KITTI/kitti_pointpillars_bag_node.py
   ```

   该节点会初始化 PointPillars 推理器并等待 `/kitti/velo/pointcloud` 点云流；需要时可改用 `MMDET3D_KITTI/run_bag_node.sh` 自动拉起 roscore、检测节点与 rosbag。
5. **播放 rosbag（终端 D，保持运行）**

   ```bash
   rosparam set use_sim_time true
   rosbag play tracking/rosbags/seq_0019_with_det.bag --clock --loop
   ```

6. **【可选】打开 RViz（终端 E，与 rosbag 同时运行）**

   ```bash
   ./Scripts/run_rviz.sh
   ```

   使用 `kitti_simple.rviz` 配置实时查看点云、PointPillars 检测与 MCTrack 结果。
7. **启动 FAST-LIO 映射（终端 F）**

   ```bash
   ./Scripts/run_fastlio_mapping.sh
   ```
8. **启动 FAST-LIO → Pose 桥接（终端 G）**

   ```bash
   ./Scripts/run_fastlio_pose_bridge.sh
   ```
9. **启动 MCTrack 在线节点（终端 H）**

   ```bash
   ./Scripts/run_mctrack_online_node.sh
   ```

   节点订阅 `/mctrack/lidar_pose` 与 `/detection/kitti_tracking`，实时发布 `/mctrack/markers`。

**【可选】一键启动检测+rosbag**

```bash
./MMDET3D_KITTI/run_bag_node.sh
```

脚本会激活 `ME5400` 环境并串行启动 PointPillars 节点与默认 bag 播放；其余 FAST-LIO/MCTrack 组件仍需按上文步骤启动（后续会整合一键脚本）。

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
