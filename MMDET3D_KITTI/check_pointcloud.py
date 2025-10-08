#!/usr/bin/env python3
"""
检查点云数据的范围和分布
"""

import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

def load_point_cloud(bin_file):
    """加载点云数据"""
    points = np.fromfile(bin_file, dtype=np.float32)
    if points.shape[0] % 4 != 0:
        points = points[:points.shape[0] - (points.shape[0] % 4)]
    return points.reshape(-1, 4)

def analyze_pointcloud(pc_file):
    """分析点云数据"""
    print(f"分析点云文件: {pc_file}")
    
    # 加载点云
    points = load_point_cloud(pc_file)
    print(f"点云大小: {points.shape}")
    
    # 分析坐标范围
    print(f"\n坐标范围:")
    print(f"  X: [{points[:, 0].min():.2f}, {points[:, 0].max():.2f}]")
    print(f"  Y: [{points[:, 1].min():.2f}, {points[:, 1].max():.2f}]")
    print(f"  Z: [{points[:, 2].min():.2f}, {points[:, 2].max():.2f}]")
    print(f"  Intensity: [{points[:, 3].min():.2f}, {points[:, 3].max():.2f}]")
    
    # 分析Z坐标分布
    z_values = points[:, 2]
    print(f"\nZ坐标统计:")
    print(f"  平均值: {z_values.mean():.2f}")
    print(f"  中位数: {np.median(z_values):.2f}")
    print(f"  标准差: {z_values.std():.2f}")
    
    # 检查是否有异常高的点
    high_points = points[points[:, 2] > 5.0]
    low_points = points[points[:, 2] < -5.0]
    print(f"\n异常点统计:")
    print(f"  Z > 5.0 的点数: {len(high_points)} ({len(high_points)/len(points)*100:.1f}%)")
    print(f"  Z < -5.0 的点数: {len(low_points)} ({len(low_points)/len(points)*100:.1f}%)")
    
    # 检查点云密度分布
    print(f"\n点云密度分析:")
    for z_level in np.arange(-3, 4, 1):
        level_points = points[(points[:, 2] >= z_level) & (points[:, 2] < z_level + 1)]
        print(f"  Z在[{z_level}, {z_level+1})的点数: {len(level_points)}")
    
    # 创建Z坐标直方图
    plt.figure(figsize=(12, 8))
    
    plt.subplot(2, 2, 1)
    plt.hist(points[:, 2], bins=50, alpha=0.7, edgecolor='black')
    plt.xlabel('Z坐标 (米)')
    plt.ylabel('点数')
    plt.title('Z坐标分布')
    plt.grid(True, alpha=0.3)
    
    plt.subplot(2, 2, 2)
    plt.scatter(points[:, 0], points[:, 1], c=points[:, 2], cmap='viridis', s=0.5, alpha=0.6)
    plt.xlabel('X坐标 (米)')
    plt.ylabel('Y坐标 (米)')
    plt.title('XY平面投影 (颜色表示Z高度)')
    plt.colorbar(label='Z坐标')
    plt.axis('equal')
    
    plt.subplot(2, 2, 3)
    plt.scatter(points[:, 0], points[:, 2], c=points[:, 1], cmap='viridis', s=0.5, alpha=0.6)
    plt.xlabel('X坐标 (米)')
    plt.ylabel('Z坐标 (米)')
    plt.title('XZ平面投影 (颜色表示Y坐标)')
    plt.colorbar(label='Y坐标')
    
    plt.subplot(2, 2, 4)
    plt.scatter(points[:, 1], points[:, 2], c=points[:, 0], cmap='viridis', s=0.5, alpha=0.6)
    plt.xlabel('Y坐标 (米)')
    plt.ylabel('Z坐标 (米)')
    plt.title('YZ平面投影 (颜色表示X坐标)')
    plt.colorbar(label='X坐标')
    
    plt.tight_layout()
    plt.savefig('pointcloud_analysis.png', dpi=150, bbox_inches='tight')
    plt.show()
    
    return points

if __name__ == "__main__":
    # 分析第一个点云文件
    pc_file = "/home/zzy/CenterPoint/data/nuScenes/samples/LIDAR_TOP/n008-2018-08-01-15-16-36-0400__LIDAR_TOP__1533151603547590.pcd.bin"
    
    if Path(pc_file).exists():
        points = analyze_pointcloud(pc_file)
    else:
        print(f"文件不存在: {pc_file}")
        
        # 尝试找到其他文件
        data_dir = "/home/zzy/CenterPoint/data/nuScenes/samples/LIDAR_TOP/"
        import glob
        files = glob.glob(os.path.join(data_dir, "*.pcd.bin"))
        if files:
            print(f"找到 {len(files)} 个点云文件，分析第一个:")
            points = analyze_pointcloud(files[0])
        else:
            print("没有找到点云文件")



