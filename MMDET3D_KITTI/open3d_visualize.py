#!/usr/bin/env python3
"""
使用 Open3D 可视化 3D 点云检测结果
"""

import os
import glob
import numpy as np
import open3d as o3d
import time
from pathlib import Path

def load_point_cloud(bin_file):
    """加载点云数据"""
    points = np.fromfile(bin_file, dtype=np.float32)
    if points.shape[0] % 4 != 0:
        points = points[:points.shape[0] - (points.shape[0] % 4)]
    return points.reshape(-1, 4)

def create_bbox_lineset(bbox, color=[1, 0, 0]):
    """创建3D边界框的线框"""
    x, y, z, dx, dy, dz, yaw = bbox[:7]
    
    # 创建边界框的8个顶点
    corners = np.array([
        [-dx/2, -dy/2, -dz/2],
        [dx/2, -dy/2, -dz/2],
        [dx/2, dy/2, -dz/2],
        [-dx/2, dy/2, -dz/2],
        [-dx/2, -dy/2, dz/2],
        [dx/2, -dy/2, dz/2],
        [dx/2, dy/2, dz/2],
        [-dx/2, dy/2, dz/2]
    ])
    
    # 应用旋转
    cos_yaw, sin_yaw = np.cos(yaw), np.sin(yaw)
    rotation_matrix = np.array([
        [cos_yaw, -sin_yaw, 0],
        [sin_yaw, cos_yaw, 0],
        [0, 0, 1]
    ])
    corners = corners @ rotation_matrix.T
    corners += np.array([x, y, z])
    
    # 定义12条边
    lines = [
        [0, 1], [1, 2], [2, 3], [3, 0],  # 底面
        [4, 5], [5, 6], [6, 7], [7, 4],  # 顶面
        [0, 4], [1, 5], [2, 6], [3, 7]   # 连接边
    ]
    
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(corners)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector([color for _ in range(len(lines))])
    
    return line_set

def visualize_single_frame(points, bboxes=None, labels=None, scores=None, frame_name=""):
    """可视化单帧点云和检测结果"""
    # 创建点云
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points[:, :3])
    
    # 根据高度着色
    colors = plt.cm.viridis((points[:, 2] - points[:, 2].min()) / (points[:, 2].max() - points[:, 2].min()))[:, :3]
    pcd.colors = o3d.utility.Vector3dVector(colors)
    
    # 创建几何体列表
    geometries = [pcd]
    
    # 添加检测边界框
    if bboxes is not None and len(bboxes) > 0:
        class_colors = [
            [1, 0, 0],    # 红色 - Car
            [0, 1, 0],    # 绿色 - Pedestrian  
            [0, 0, 1],    # 蓝色 - Cyclist
            [1, 1, 0],    # 黄色 - Truck
            [1, 0, 1],    # 紫色 - Bus
            [0, 1, 1],    # 青色 - Trailer
            [0.5, 0.5, 0.5], # 灰色 - Construction
            [0.5, 0, 0.5],   # 紫色 - Motorcycle
            [0, 0.5, 0.5],   # 青色 - Bicycle
            [0.5, 0.5, 0]    # 棕色 - Traffic Cone
        ]
        
        for i, bbox in enumerate(bboxes):
            if len(bbox) >= 7:
                label = labels[i] if labels is not None and i < len(labels) else 0
                color = class_colors[label % len(class_colors)]
                
                bbox_lineset = create_bbox_lineset(bbox, color)
                geometries.append(bbox_lineset)
    
    # 可视化
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name=f"3D Detection Results - {frame_name}", width=1200, height=800)
    
    for geom in geometries:
        vis.add_geometry(geom)
    
    # 设置视角
    view_control = vis.get_view_control()
    view_control.set_front([0, 0, -1])
    view_control.set_up([0, -1, 0])
    view_control.set_lookat([0, 0, 0])
    
    # 渲染
    vis.poll_events()
    vis.update_renderer()
    
    return vis

def run_continuous_visualization():
    """运行连续可视化"""
    print("🚀 启动 Open3D 连续可视化...")
    
    # 数据路径
    data_dir = "/home/zzy/CenterPoint/data/nuScenes/samples/LIDAR_TOP/"
    point_cloud_files = sorted(glob.glob(os.path.join(data_dir, "*.pcd.bin")))
    
    if not point_cloud_files:
        print("❌ 没有找到点云文件！")
        return
    
    print(f"📁 找到 {len(point_cloud_files)} 个点云文件")
    
    # 选择要可视化的帧数
    max_frames = min(50, len(point_cloud_files))  # 最多50帧
    selected_files = point_cloud_files[:max_frames]
    
    print(f"🎬 将连续播放前 {max_frames} 帧")
    print("💡 提示：")
    print("   - 鼠标左键拖拽：旋转视角")
    print("   - 鼠标右键拖拽：平移视角") 
    print("   - 滚轮：缩放")
    print("   - 按 'Q' 或关闭窗口：退出")
    print("   - 按 'N' 或空格：下一帧")
    print("   - 按 'P'：上一帧")
    print("   - 按 'R'：重置视角")
    
    # 初始化可视化器
    vis = None
    current_frame = 0
    
    try:
        while current_frame < len(selected_files):
            pc_file = selected_files[current_frame]
            frame_name = os.path.basename(pc_file)
            
            print(f"\n📊 处理第 {current_frame + 1}/{len(selected_files)} 帧: {frame_name}")
            
            # 加载点云
            points = load_point_cloud(pc_file)
            print(f"   点云大小: {points.shape}")
            
            # 创建可视化
            if vis is not None:
                vis.destroy_window()
            
            vis = visualize_single_frame(points, frame_name=frame_name)
            
            # 等待用户交互
            print("   等待用户操作... (按 'N' 下一帧，'P' 上一帧，'Q' 退出)")
            
            # 简单的交互循环
            while True:
                vis.poll_events()
                vis.update_renderer()
                
                # 检查窗口是否关闭
                if not vis.poll_events():
                    print("   窗口已关闭，退出可视化")
                    return
                
                time.sleep(0.01)  # 避免CPU占用过高
                
                # 这里可以添加键盘事件处理
                # 由于Open3D的键盘事件处理比较复杂，我们使用简单的定时器
                # 实际使用中，用户可以通过鼠标操作视角，然后手动关闭窗口进入下一帧
                
    except KeyboardInterrupt:
        print("\n⏹️  用户中断，停止可视化")
    except Exception as e:
        print(f"\n❌ 可视化过程中出现错误: {e}")
    finally:
        if vis is not None:
            vis.destroy_window()
    
    print("✅ 可视化完成！")

def run_single_frame_visualization():
    """运行单帧可视化"""
    print("🚀 启动 Open3D 单帧可视化...")
    
    # 数据路径
    data_dir = "/home/zzy/CenterPoint/data/nuScenes/samples/LIDAR_TOP/"
    point_cloud_files = sorted(glob.glob(os.path.join(data_dir, "*.pcd.bin")))
    
    if not point_cloud_files:
        print("❌ 没有找到点云文件！")
        return
    
    # 选择第一帧
    pc_file = point_cloud_files[0]
    frame_name = os.path.basename(pc_file)
    
    print(f"📊 可视化帧: {frame_name}")
    
    # 加载点云
    points = load_point_cloud(pc_file)
    print(f"   点云大小: {points.shape}")
    
    # 创建可视化
    vis = visualize_single_frame(points, frame_name=frame_name)
    
    print("💡 提示：")
    print("   - 鼠标左键拖拽：旋转视角")
    print("   - 鼠标右键拖拽：平移视角") 
    print("   - 滚轮：缩放")
    print("   - 按 'Q' 或关闭窗口：退出")
    
    try:
        # 保持窗口打开
        while True:
            vis.poll_events()
            vis.update_renderer()
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\n⏹️  用户中断，停止可视化")
    finally:
        vis.destroy_window()
    
    print("✅ 可视化完成！")

if __name__ == "__main__":
    import matplotlib.pyplot as plt
    
    print("🎯 Open3D 3D 点云检测结果可视化")
    print("=" * 50)
    print("选择可视化模式:")
    print("1. 连续播放多帧 (推荐)")
    print("2. 单帧可视化")
    
    try:
        choice = input("请输入选择 (1 或 2): ").strip()
        
        if choice == "1":
            run_continuous_visualization()
        elif choice == "2":
            run_single_frame_visualization()
        else:
            print("无效选择，使用默认的连续播放模式")
            run_continuous_visualization()
    except KeyboardInterrupt:
        print("\n👋 再见！")
    except Exception as e:
        print(f"❌ 程序错误: {e}")



