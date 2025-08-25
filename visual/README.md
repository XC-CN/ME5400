# MCTrack 可视化工具

本目录包含MCTrack的实时可视化工具，可以显示车辆运动和跟踪效果。

## 📁 文件说明

### 1. `realtime_visualizer.py` - 独立实时可视化工具
- **功能**: 从跟踪结果文件读取数据，提供实时播放可视化
- **特点**: 
  - 实时FPS显示
  - 车辆轨迹历史
  - 性能统计面板
  - 支持视频录制

### 2. `integrated_visualizer.py` - 集成可视化模块
- **功能**: 直接集成到MCTrack主程序中的高性能可视化
- **特点**:
  - 异步渲染，不影响跟踪性能
  - 实时显示跟踪过程
  - 内存优化的轨迹显示

### 3. `visualize_tracking.py` - 2D可视化工具
- **功能**: 简单的2D俯视图可视化
- **特点**: 轻量级，适合快速预览

### 4. `visualize_3d.py` - 3D可视化工具
- **功能**: Open3D 3D点云可视化
- **特点**: 立体显示，支持交互操作

## 🚀 快速开始

### 方法1: 实时可视化演示（推荐）
```bash
# 1. 首先运行MCTrack生成结果
cd D:\OneDrive\NUS\ME5400\MCTrack
conda activate MCTrack
python main.py --dataset kitti -e -p 1

# 2. 运行实时可视化
python visual/realtime_visualizer.py
```

### 方法2: 3D可视化（需要Open3D）
```bash
# 安装Open3D
pip install open3d

# 运行3D可视化
python visual/visualize_3d.py --result_path results/kitti/val_car_summary.json
```

### 方法3: 集成到跟踪过程
在 `main.py` 中添加：
```python
from visual.integrated_visualizer import create_visualizer, visualize_tracking_frame

# 创建可视化器
visualizer = create_visualizer(config, enable_visualization=True)

# 在跟踪循环中
visualize_tracking_frame(visualizer, trajectories, detections, scene_info)

# 结束时清理
if visualizer:
    visualizer.cleanup()
```

## ⌨️ 控制键

- **q**: 退出
- **空格**: 暂停/继续
- **r**: 重置（适用部分工具）
- **n**: 下一帧（适用部分工具）

## 📊 显示内容

### 实时性能指标
- **FPS**: 当前帧率
- **Active Tracks**: 活跃轨迹数量
- **Avg Detections**: 平均检测数量
- **Processing Time**: 处理时间

### 车辆信息
- **彩色矩形**: 不同颜色代表不同车辆ID
- **白色箭头**: 车辆朝向
- **ID标签**: 轨迹ID号
- **轨迹线**: 历史运动轨迹（渐变透明度）
- **速度信息**: 车辆当前速度

### 坐标系
- **红色箭头**: X轴方向
- **绿色箭头**: Y轴方向
- **网格**: 帮助判断距离和位置

## 🔧 自定义配置

### 修改可视化参数
编辑相应的Python文件：

```python
# 画布大小
self.canvas_size = (1600, 1200)

# 坐标缩放
self.scale = 8

# 轨迹历史长度
maxlen=50

# 播放速度
time.sleep(0.05)  # 20 FPS
```

### 录制视频
```python
# 开启视频录制
visualizer = RealTimeVisualizer(save_video=True)
```

## 🎯 最佳实践

### 查看实时跟踪效果
1. 使用 `realtime_visualizer.py` 查看整体跟踪效果
2. 观察轨迹连续性和车辆ID一致性
3. 监控FPS性能

### 调试跟踪算法
1. 使用 `integrated_visualizer.py` 集成到开发流程
2. 实时观察参数调整效果
3. 记录性能变化

### 制作演示视频
1. 开启视频录制功能
2. 选择合适的场景和参数
3. 后期可以添加说明文字

## 🔍 故障排除

### 常见问题

**1. "No tracking results found"**
- 解决：先运行 `python main.py --dataset kitti -e -p 1`

**2. Open3D导入错误**
- 解决：`pip install open3d`

**3. 可视化窗口无响应**
- 解决：检查OpenCV安装，或使用集成可视化

**4. 视频录制失败**
- 解决：确保有写入权限，检查磁盘空间

### 性能优化

- 降低画布分辨率
- 减少轨迹历史长度
- 使用异步渲染
- 关闭不必要的显示元素

## 📝 开发说明

### 添加新的可视化功能
1. 继承基础可视化类
2. 实现自定义渲染方法
3. 添加相应的配置选项

### 集成到其他项目
1. 复制 `visual/` 目录
2. 修改导入路径
3. 适配数据格式

欢迎贡献更多可视化功能！