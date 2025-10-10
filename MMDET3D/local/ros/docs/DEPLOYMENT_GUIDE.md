# KITTI PointPillars ROS节点部署指南

## 📦 部署包内容

这个部署包包含了运行KITTI PointPillars ROS节点所需的所有文件：

```
MMDET3D/local/ros/deployment/
├── kitti_pointpillars_ros_node.py    # 打包用主ROS节点副本
├── launch/                           # ROS启动文件
│   └── kitti_pointpillars.launch
├── rviz/                             # RViz配置文件
│   └── kitti_detection.rviz
├── pointpillars/                     # PointPillars配置文件快照
│   └── pointpillars_hv_secfpn_8xb6-160e_kitti-3d-3class.py
├── checkpoints/                      # 预训练模型（与根目录 checkpoints 相同）
├── test_ros_node.py                  # 测试脚本
├── README_ROS_NODE.md                # 详细使用说明
└── DEPLOYMENT_GUIDE.md               # 本文件
```

## 🚀 快速开始

### 1. 环境要求

- **Python**: 3.8+
- **ROS**: Noetic
- **CUDA**: 11.0+ (用于GPU推理)
- **MMDetection3D**: 已安装并配置

⚠️ **重要**: 这个部署包需要完整的MMDetection3D环境，包括：
- `mmdet3d` 包已安装
- `mmengine` 包已安装
- `torch` 和 `torchvision` 已安装
- CUDA环境配置正确

### 2. 安装依赖

```bash
# 激活MMDetection3D环境（项目统一使用 ME5400）
conda activate ME5400

# 安装ROS依赖
pip install rospy rospkg tf
```

### 3. 准备KITTI数据

将KITTI数据放在以下目录结构：
```
data/kitti/2011_09_26_drive_0039_sync/2011_09_26/2011_09_26_drive_0039_sync/velodyne_points/data/
├── 0000000000.bin
├── 0000000001.bin
├── 0000000002.bin
└── ...
```

### 4. 运行ROS节点

```bash
# 启动ROS master
roscore &

# 运行ROS节点
python MMDET3D/local/ros/nodes/kitti_pointpillars_ros_node.py --mode continuous

# 或者使用launch文件（绝对路径）
roslaunch $(pwd)/MMDET3D/local/ros/launch/kitti_pointpillars.launch
```

### 5. 可视化

```bash
# 启动RViz
rviz -d $(pwd)/MMDET3D/local/ros/rviz/kitti_detection.rviz
```

## 📊 ROS话题

节点发布以下话题：

- `/detection/bboxes_3d` (visualization_msgs/MarkerArray) - 3D边界框可视化
- `/detection/kitti_tracking` (std_msgs/String) - KITTI tracking格式检测结果
- `/detection/status` (std_msgs/String) - 检测状态信息

## 🔧 配置参数

可以通过ROS参数调整：

- `config_path`: 配置文件路径
- `checkpoint_path`: 模型检查点路径
- `data_dir`: KITTI数据目录
- `publish_rate`: 发布频率 (Hz)
- `confidence_threshold`: 置信度阈值
- `frame_id`: 坐标系ID
- `mode`: 运行模式 (continuous/single)
- `frame_idx`: 单帧模式下的帧索引

## 🧪 测试

运行测试脚本验证安装：

```bash
python MMDET3D/local/ros/tests/test_ros_node.py
```

## 📝 注意事项

1. **GPU内存**: 确保有足够的GPU内存运行推理
2. **数据格式**: 确保KITTI数据格式正确
3. **坐标系**: 默认使用KITTI坐标系 (x前, y左, z上)
4. **性能**: 推理速度取决于GPU性能和数据大小

## 🆘 故障排除

### 常见问题

1. **CUDA内存不足**
   ```bash
   # 检查GPU使用情况
   nvidia-smi
   # 杀死其他Python进程释放内存
   pkill python
   ```

2. **找不到模型文件**
   - 检查checkpoint_path参数
   - 确保.pth文件存在

3. **ROS话题无数据**
   - 检查ROS master是否运行
   - 验证数据目录路径
   - 查看节点日志输出

## 📞 支持

如有问题，请检查：
1. README_ROS_NODE.md 详细文档
2. 节点日志输出
3. ROS话题状态

---

**版本**: 1.0  
**更新日期**: 2024-10-03  
**兼容性**: MMDetection3D + ROS Noetic
