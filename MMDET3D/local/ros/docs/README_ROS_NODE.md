# KITTI PointPillars ROS节点使用说明

## 概述

这个ROS节点将MMDetection3D PointPillars检测结果转换为ROS MarkerArray消息，供其他ROS节点使用和RViz可视化。

## 功能特性

- **连续推理**: 处理KITTI序列数据，连续发布检测结果
- **单帧推理**: 处理单个点云文件
- **ROS消息发布**: 发布MarkerArray格式的3D边界框
- **RViz可视化**: 支持在RViz中显示3D检测结果
- **参数配置**: 支持ROS参数服务器配置

## 文件结构

```
MMDET3D/local/ros/
├── nodes/
│   ├── kitti_pointpillars_ros_node.py   # 主ROS节点
│   └── kitti_pointpillars_bag_node.py   # rosbag 输入节点
├── launch/                              # 示例 launch 文件
├── rviz/                                # RViz 配置
├── scripts/run_bag_node.sh              # 一键启动脚本
├── tests/test_ros_node.py               # 话题发布验证脚本
└── docs/                                # 使用文档（本文档等）
```

## 安装依赖

### ROS依赖
```bash
# 安装ROS可视化消息包
sudo apt-get install ros-noetic-visualization-msgs
sudo apt-get install ros-noetic-tf
```

### Python依赖
```bash
# 在 ME5400 环境中安装
conda activate ME5400
pip install rospy tf
```

## 使用方法

### 1. 启动ROS核心
```bash
roscore
```

### 2. 启动检测节点

#### 连续推理模式
```bash
roslaunch mmdet3d_centerpoint kitti_pointpillars.launch mode:=continuous
```

#### 单帧推理模式
```bash
roslaunch mmdet3d_centerpoint kitti_pointpillars.launch mode:=single frame_idx:=0
```

#### 自定义参数
```bash
roslaunch mmdet3d_centerpoint kitti_pointpillars.launch \
    config_path:=/path/to/config.py \
    checkpoint_path:=/path/to/checkpoint.pth \
    data_dir:=/path/to/kitti/data \
    publish_rate:=10.0 \
    confidence_threshold:=0.1 \
    frame_id:=lidar
```

### 3. 启动RViz可视化
```bash
roslaunch mmdet3d_centerpoint kitti_pointpillars.launch rviz:=true
```

或者手动启动RViz：
```bash
rviz -d $(pwd)/MMDET3D/local/ros/rviz/kitti_detection.rviz
```

### 4. 测试节点
```bash
python MMDET3D/local/ros/tests/test_ros_node.py
```

## ROS话题

### 发布的话题

| 话题名称 | 消息类型 | 描述 |
|---------|---------|------|
| `/detection/bboxes_3d` | `visualization_msgs/MarkerArray` | 3D检测边界框 |
| `/detection/status` | `std_msgs/String` | 检测状态信息 |

### MarkerArray消息格式

每个Marker包含：
- **位置**: 3D边界框中心坐标 (x, y, z)
- **姿态**: 旋转四元数 (基于yaw角度)
- **尺寸**: 长度、宽度、高度 (l, w, h)
- **颜色**: 根据类别着色
  - 红色: Car
  - 绿色: Pedestrian  
  - 蓝色: Cyclist

## 参数配置

### ROS参数

| 参数名 | 类型 | 默认值 | 描述 |
|-------|------|--------|------|
| `config_path` | string | 配置文件路径 | MMDetection3D模型配置 |
| `checkpoint_path` | string | 检查点路径 | 预训练模型权重 |
| `data_dir` | string | 数据目录 | KITTI点云数据目录 |
| `publish_rate` | float | 10.0 | 发布频率 (Hz) |
| `confidence_threshold` | float | 0.1 | 置信度阈值 |
| `frame_id` | string | "lidar" | 坐标系ID |
| `mode` | string | "continuous" | 运行模式 |
| `frame_idx` | int | 0 | 单帧模式下的帧索引 |

## 坐标系

- **坐标系**: `lidar` (LiDAR坐标系)
- **坐标轴**: 
  - X: 前方 (车辆前进方向)
  - Y: 左侧
  - Z: 上方
- **单位**: 米 (m)

## 检测结果格式

### 输入格式 (MMDetection3D)
```python
{
    "labels_3d": [0, 0, 0, ...],           # 类别标签
    "scores_3d": [0.73, 0.68, 0.57, ...], # 置信度分数
    "bboxes_3d": [                         # 3D边界框
        [x, y, z, l, w, h, yaw, vx, vy],   # [x, y, z, 长, 宽, 高, 偏航角, vx, vy]
        ...
    ],
    "box_type_3d": "LiDAR"
}
```

### 输出格式 (ROS MarkerArray)
```python
{
    "markers": [
        {
            "header": {"frame_id": "lidar", "stamp": "..."},
            "ns": "detection",
            "id": 0,
            "type": 1,  # CUBE
            "action": 0,  # ADD
            "pose": {
                "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
            },
            "scale": {"x": 4.0, "y": 2.0, "z": 1.5},
            "color": {"r": 1.0, "g": 0.0, "b": 0.0, "a": 0.8},
            "lifetime": {"secs": 0, "nsecs": 0},
            "frame_locked": false
        }
    ]
}
```

## 故障排除

### 常见问题

1. **模型加载失败**
   - 检查配置文件路径是否正确
   - 检查检查点文件是否存在
   - 确认MMDetection3D环境正确安装

2. **数据加载失败**
   - 检查数据目录路径是否正确
   - 确认.bin文件存在且可读
   - 检查文件权限

3. **ROS话题无数据**
   - 检查roscore是否运行
   - 确认节点启动成功
   - 使用`rostopic list`检查话题

4. **RViz无显示**
   - 检查frame_id设置
   - 确认MarkerArray话题正确
   - 检查RViz配置

### 调试命令

```bash
# 检查ROS话题
rostopic list
rostopic echo /detection/bboxes_3d
rostopic echo /detection/status

# 检查节点状态
rosnode list
rosnode info /kitti_pointpillars_detector

# 检查参数
rosparam list
rosparam get /kitti_pointpillars_detector/config_path
```

## 扩展使用

### 与其他ROS节点集成

1. **路径规划**: 将检测结果用于障碍物避让
2. **SLAM**: 将检测结果用于地图构建
3. **控制**: 将检测结果用于车辆控制
4. **记录**: 使用rosbag记录检测结果

### 自定义配置

1. **修改检测类别**: 编辑`class_names`和`class_colors`
2. **调整可视化**: 修改Marker属性
3. **添加新功能**: 扩展节点功能

## 性能优化

- **GPU加速**: 确保CUDA环境正确配置
- **内存管理**: 调整批处理大小
- **发布频率**: 根据需求调整发布频率
- **置信度阈值**: 平衡检测精度和速度

## 许可证

本项目基于MMDetection3D许可证。
