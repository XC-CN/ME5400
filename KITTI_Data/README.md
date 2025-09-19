# KITTI数据集文件夹

本文件夹包含了KITTI数据集相关的所有文件和工具。

## 📁 文件结构

```
KITTI_Data/
├── 2011_09_26_drive_0019_sync.zip     # KITTI原始数据包 (2GB)
├── 2011_09_26/                        # 解压后的原始数据
│   └── 2011_09_26_drive_0019_sync/
│       ├── velodyne_points/data/       # Velodyne激光雷达数据 (.bin文件)
│       ├── oxts/data/                  # IMU/GPS数据 (.txt文件)
│       └── image_*/                    # 相机图像数据
├── kitti_2011_09_26_drive_0019_sync.bag # 转换后的ROS bag文件 (913MB)
├── kitti_to_rosbag.py                  # 数据转换脚本
└── README.md                           # 本说明文件
```

## 📊 数据统计

- **序列**: 2011_09_26_drive_0019_sync
- **帧数**: 481帧
- **激光雷达**: Velodyne HDL-64E (64线)
- **IMU频率**: 100Hz
- **激光雷达频率**: 10Hz

## 🔧 使用方法

### 1. 解压原始数据
```bash
cd KITTI_Data
unzip 2011_09_26_drive_0019_sync.zip
```

### 2. 转换为ROS bag格式
```bash
# 如果需要重新生成bag文件
python3 kitti_to_rosbag.py
```

### 3. 在FAST-LIO2中使用
```bash
# 回到项目根目录
cd ../catkin_ws
source devel/setup.bash

# 启动FAST-LIO2
roslaunch fast_lio mapping_velodyne.launch

# 播放数据（新终端）
rosbag play ../KITTI_Data/kitti_2011_09_26_drive_0019_sync.bag
```

## 📝 数据格式说明

### Velodyne点云数据 (.bin)
- 每个点包含: x, y, z, intensity (4个float32值)
- 文件命名: 0000000000.bin, 0000000001.bin, ...

### OXTS IMU数据 (.txt)
- 包含: 纬度, 经度, 高度, roll, pitch, yaw, 速度, 加速度, 角速度等
- 30个数值，空格分隔

### ROS bag话题
- `/kitti/velo/pointcloud`: sensor_msgs/PointCloud2
- `/kitti/oxts/imu`: sensor_msgs/Imu

## ⚠️ 注意事项

1. **大文件**: 原始数据和bag文件都很大，已在.gitignore中排除
2. **磁盘空间**: 确保有足够空间存储解压后的数据
3. **路径**: 脚本中的路径可能需要根据实际情况调整

## 🔗 相关链接

- [KITTI数据集官网](http://www.cvlibs.net/datasets/kitti/)
- [KITTI原始数据下载](http://www.cvlibs.net/datasets/kitti/raw_data.php)
