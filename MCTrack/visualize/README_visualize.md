# nuScenes LiDAR 可视化工具

这个文件夹包含了用于在Rviz中可视化nuScenes LiDAR数据的完整工具集。

## 📁 文件说明

### 1. `start_lidar_visualization.sh` - 一键启动脚本
- **作用**: 自动化启动整个可视化系统
- **功能**:
  - 检查并启动ROS核心
  - 激活conda环境
  - 自动启动Rviz（使用预配置）
  - 启动LiDAR数据播放器
  - 自动清理资源

### 2. `nuscenes_rviz_visualizer.py` - 核心可视化引擎
- **作用**: 数据处理和ROS消息发布
- **功能**:
  - 加载nuScenes数据集
  - 连续播放LiDAR点云数据
  - 发布ROS话题: `/nuscenes/lidar`
  - 发布TF变换: `map` → `lidar`
  - 播放频率: 2Hz（与数据集原始频率一致）

### 3. `nuscenes_lidar.rviz` - Rviz配置文件
- **作用**: 预配置的Rviz显示设置
- **配置内容**:
  - PointCloud2显示已添加
  - 话题设置为: `/nuscenes/lidar`
  - 坐标系设置为: `lidar`
  - 固定坐标系设置为: `map`
  - 点云颜色基于强度值

## 🚀 使用方法

### 方法1: 一键启动（推荐）
```bash
cd /home/xc/Projects/ME5400/ME5400
./visualize/start_lidar_visualization.sh
```

### 方法2: 手动启动
```bash
# 1. 启动ROS核心
roscore &

# 2. 启动Rviz
rviz -d visualize/nuscenes_lidar.rviz &

# 3. 启动LiDAR播放器
source /home/xc/miniconda3/bin/activate MCTrack
python3 visualize/nuscenes_rviz_visualizer.py /home/xc/Projects/ME5400/ME5400/data/nuScenes
```

## 📊 数据说明

- **数据集**: nuScenes v1.0-trainval
- **场景数量**: 850个场景
- **播放频率**: 2Hz（每0.5秒一帧）
- **每个场景**: 约40个样本，持续20秒
- **总播放时间**: 约4.7小时

## 🎯 预期效果

在Rviz中您将看到：
- 连续的3D LiDAR点云数据
- 点云随时间变化，显示车辆行驶过程
- 点云颜色基于激光强度值
- 控制台显示当前播放进度

## ⚠️ 注意事项

1. 确保数据集路径正确: `/home/xc/Projects/ME5400/ME5400/data/nuScenes`
2. 确保conda环境`MCTrack`已安装
3. 确保ROS Noetic已安装
4. 按`Ctrl+C`停止播放

## 🔧 故障排除

如果遇到问题：
1. 检查ROS核心是否运行: `rostopic list`
2. 检查数据集是否存在: `ls /home/xc/Projects/ME5400/ME5400/data/nuScenes`
3. 检查conda环境: `conda activate MCTrack`
4. 查看错误信息并相应处理


## 📚 连续长段（同一log）推荐与使用方法

为了进行长时间连续回放（便于 MCTrack 测试），建议选择“同一 log 的一组场景”。同一 log 表示来自同一次采集，跨场景播放时时间关系更连贯。

- 如何使用：
  - 打开 `visualize/nuscenes_rviz_visualizer.py` 顶部配置，将 `START_SCENE_INDEX` 设置为下方列表中对应“起始场景索引”的值，即可从该长段第一个场景开始播放。
  - 播放帧率可通过 `PLAYBACK_HZ` 调整。

- 基于当前本地数据（trainval_blobz_01，且仅统计本地 LiDAR_TOP 文件均存在的场景），按样本总数降序的连续分组TOP示例：
  - [1] 地点: singapore-onenorth | 场景数: 16 | 样本总数: 637 | 起止场景索引: 18 -> 33 | 起止名称: scene-0019 -> scene-0034
  - [2] 地点: boston-seaport | 场景数: 15 | 样本总数: 595 | 起止场景索引: 59 -> 73 | 起止名称: scene-0062 -> scene-0076
  - [3] 地点: boston-seaport | 场景数: 11 | 样本总数: 434 | 起止场景索引: 74 -> 84 | 起止名称: scene-0092 -> scene-0102
  - [4] 地点: singapore-onenorth | 场景数: 10 | 样本总数: 393 | 起止场景索引: 42 -> 51 | 起止名称: scene-0045 -> scene-0054
  - [5] 地点: singapore-onenorth | 场景数: 8 | 样本总数: 318 | 起止场景索引: 3 -> 10 | 起止名称: scene-0004 -> scene-0011

说明：上述均为“同一 log”的连续段，适合长段联播。若需重新统计，可参考我们在终端运行的分组脚本（按 log 分组、统计样本数并排序）。
