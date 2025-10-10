#!/usr/bin/env python3
"""
分析KITTI数据格式和检测结果
"""

import numpy as np
import json

def analyze_kitti_data():
    """分析KITTI数据格式"""
    
    # 加载点云数据
    bin_path = 'data/kitti/2011_09_26_drive_0039_sync/2011_09_26/2011_09_26_drive_0039_sync/velodyne_points/data/0000000000.bin'
    points = np.fromfile(bin_path, dtype=np.float32).reshape(-1, 4)
    
    print("=== KITTI点云数据分析 ===")
    print(f"点云数量: {len(points)}")
    print(f"点云形状: {points.shape}")
    print(f"点云范围:")
    print(f"  X: [{points[:, 0].min():.2f}, {points[:, 0].max():.2f}]")
    print(f"  Y: [{points[:, 1].min():.2f}, {points[:, 1].max():.2f}]")
    print(f"  Z: [{points[:, 2].min():.2f}, {points[:, 2].max():.2f}]")
    print(f"  Intensity: [{points[:, 3].min():.2f}, {points[:, 3].max():.2f}]")
    
    # 加载检测结果
    result_path = 'data/kitti/2011_09_26_drive_0039_sync/2011_09_26/2011_09_26_drive_0039_sync/velodyne_points/data/0000000000_pointpillars_result.json'
    with open(result_path, 'r') as f:
        result = json.load(f)
    
    print("\n=== 检测结果分析 ===")
    print(f"检测到 {len(result['bboxes_3d'])} 个目标")
    
    for i, (bbox, score, label, class_name) in enumerate(zip(
        result['bboxes_3d'], 
        result['scores_3d'], 
        result['labels_3d'],
        result['class_names']
    )):
        x, y, z, w, l, h, yaw = bbox
        print(f"\n目标 {i+1}: {class_name}")
        print(f"  位置: ({x:.2f}, {y:.2f}, {z:.2f})")
        print(f"  尺寸: 宽={w:.2f}m, 长={l:.2f}m, 高={h:.2f}m")
        print(f"  旋转: {yaw:.2f} 弧度 ({np.degrees(yaw):.1f}度)")
        print(f"  置信度: {score:.3f}")
        
        # 检查边界框是否在点云范围内
        in_range = (0 <= x <= 70.4 and -40 <= y <= 40 and -3 <= z <= 1)
        print(f"  在检测范围内: {in_range}")
        
        # 检查尺寸合理性
        if class_name == 'Car':
            reasonable_size = (1.5 <= w <= 4.0 and 3.0 <= l <= 6.0 and 1.0 <= h <= 2.0)
        elif class_name == 'Pedestrian':
            reasonable_size = (0.3 <= w <= 1.0 and 0.3 <= l <= 1.0 and 1.0 <= h <= 2.0)
        elif class_name == 'Cyclist':
            reasonable_size = (0.5 <= w <= 1.5 and 1.0 <= l <= 2.5 and 1.0 <= h <= 2.0)
        else:
            reasonable_size = True
            
        print(f"  尺寸合理: {reasonable_size}")

if __name__ == '__main__':
    analyze_kitti_data()



