#!/usr/bin/env python3
"""
测试KITTI可视化 - 单帧版本
"""

import os
import sys
import numpy as np
import open3d as o3d

# 添加 MMDetection3D 路径
sys.path.append('/home/zzy/mmdet3d_centerpoint')
from mmdet3d.apis import LidarDet3DInferencer
from mmengine.registry import init_default_scope

def test_visualization():
    """测试可视化"""
    print("测试Open3D可视化...")
    
    # 检查显示环境
    display = os.environ.get('DISPLAY')
    print(f"DISPLAY环境变量: {display}")
    
    # 创建简单的点云
    print("创建测试点云...")
    points = np.random.rand(1000, 3) * 10  # 1000个随机点
    
    # 创建点云对象
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.paint_uniform_color([1.0, 0.0, 0.0])  # 红色
    
    # 创建可视化器
    print("创建可视化器...")
    vis = o3d.visualization.Visualizer()
    
    # 创建窗口
    print("创建窗口...")
    if not vis.create_window(window_name="Test Visualization", width=800, height=600):
        print("错误: 无法创建窗口")
        return False
    
    print("窗口创建成功!")
    
    # 添加点云
    vis.add_geometry(pcd)
    
    # 设置视角
    ctr = vis.get_view_control()
    ctr.set_front([0, 0, 1])
    ctr.set_up([0, 1, 0])
    ctr.set_lookat([0, 0, 0])
    ctr.set_zoom(0.8)
    
    print("开始渲染...")
    print("按任意键关闭窗口...")
    
    # 运行可视化
    vis.run()
    vis.destroy_window()
    
    print("测试完成")
    return True

if __name__ == '__main__':
    test_visualization()



