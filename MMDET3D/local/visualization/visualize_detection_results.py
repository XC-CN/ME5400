#!/usr/bin/env python3
"""
使用 Open3D 可视化实际的检测结果
"""

import os
import glob
import json
import numpy as np
import open3d as o3d
import time
from pathlib import Path

def load_point_cloud(bin_file):
    """加载点云数据 - nuScenes 格式是5维的"""
    points = np.fromfile(bin_file, dtype=np.float32)
    if points.shape[0] % 5 != 0:
        points = points[:points.shape[0] - (points.shape[0] % 5)]
    return points.reshape(-1, 5)

def load_detection_results(json_file):
    """加载检测结果"""
    with open(json_file, 'r') as f:
        data = json.load(f)
    return data

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

def visualize_detection_results(points, detection_data, frame_name=""):
    """可视化点云和检测结果"""
    # 创建点云
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points[:, :3])
    
    # 根据高度着色
    colors = plt.cm.viridis((points[:, 2] - points[:, 2].min()) / (points[:, 2].max() - points[:, 2].min()))[:, :3]
    pcd.colors = o3d.utility.Vector3dVector(colors)
    
    # 创建几何体列表
    geometries = [pcd]
    
    # 获取检测结果
    if 'bboxes_3d' in detection_data and 'labels_3d' in detection_data and 'scores_3d' in detection_data:
        bboxes = detection_data['bboxes_3d']
        labels = detection_data['labels_3d']
        scores = detection_data['scores_3d']
        
        print(f"检测到 {len(bboxes)} 个目标")
        
        # nuScenes 类别名称
        class_names = [
            'car', 'truck', 'construction_vehicle', 'bus', 'trailer',
            'barrier', 'motorcycle', 'bicycle', 'pedestrian', 'traffic_cone'
        ]
        
        # 类别颜色
        class_colors = [
            [1, 0, 0],    # 红色 - car
            [0, 1, 0],    # 绿色 - truck
            [0, 0, 1],    # 蓝色 - construction_vehicle
            [1, 1, 0],    # 黄色 - bus
            [1, 0, 1],    # 紫色 - trailer
            [0, 1, 1],    # 青色 - barrier
            [0.5, 0.5, 0.5], # 灰色 - motorcycle
            [0.5, 0, 0.5],   # 紫色 - bicycle
            [0, 0.5, 0.5],   # 青色 - pedestrian
            [0.5, 0.5, 0]    # 棕色 - traffic_cone
        ]
        
        # 添加检测边界框
        for i, bbox in enumerate(bboxes):
            if len(bbox) >= 7:
                label = labels[i] if i < len(labels) else 0
                score = scores[i] if i < len(scores) else 0.0
                
                # 只显示置信度较高的检测结果
                if score > 0.1:  # 置信度阈值
                    color = class_colors[label % len(class_colors)]
                    bbox_lineset = create_bbox_lineset(bbox, color)
                    geometries.append(bbox_lineset)
                    
                    # 打印检测信息
                    class_name = class_names[label] if label < len(class_names) else f"Class_{label}"
                    print(f"  目标 {i+1}: {class_name}, 置信度={score:.3f}")
                    print(f"    位置: x={bbox[0]:.2f}, y={bbox[1]:.2f}, z={bbox[2]:.2f}")
                    print(f"    尺寸: 长={bbox[3]:.2f}, 宽={bbox[4]:.2f}, 高={bbox[5]:.2f}")
                    print(f"    旋转: {bbox[6]:.2f} 弧度")
    
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
    print("🚀 启动 Open3D 检测结果可视化...")
    
    # 数据路径
    data_dir = "/home/zzy/CenterPoint/data/nuScenes/samples/LIDAR_TOP/"
    results_dir = "/home/zzy/mmdet3d_centerpoint/all_frames_results/preds/"
    
    # 获取点云文件
    point_cloud_files = sorted(glob.glob(os.path.join(data_dir, "*.pcd.bin")))
    
    if not point_cloud_files:
        print("❌ 没有找到点云文件！")
        return
    
    print(f"📁 找到 {len(point_cloud_files)} 个点云文件")
    
    # 选择要可视化的帧数
    max_frames = min(20, len(point_cloud_files))  # 最多20帧
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
            
            # 查找对应的检测结果
            json_file = os.path.join(results_dir, frame_name.replace('.pcd.bin', '.pcd.json'))
            detection_data = None
            
            if os.path.exists(json_file):
                try:
                    detection_data = load_detection_results(json_file)
                    print(f"   加载检测结果: {json_file}")
                except Exception as e:
                    print(f"   加载检测结果失败: {e}")
            else:
                print(f"   未找到检测结果文件: {json_file}")
            
            # 创建可视化
            if vis is not None:
                vis.destroy_window()
            
            vis = visualize_detection_results(points, detection_data or {}, frame_name)
            
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
    print("🚀 启动 Open3D 单帧检测结果可视化...")
    
    # 数据路径
    data_dir = "/home/zzy/CenterPoint/data/nuScenes/samples/LIDAR_TOP/"
    results_dir = "/home/zzy/mmdet3d_centerpoint/all_frames_results/preds/"
    
    # 选择第一帧
    point_cloud_files = sorted(glob.glob(os.path.join(data_dir, "*.pcd.bin")))
    if not point_cloud_files:
        print("❌ 没有找到点云文件！")
        return
    
    pc_file = point_cloud_files[0]
    frame_name = os.path.basename(pc_file)
    
    print(f"📊 可视化帧: {frame_name}")
    
    # 加载点云
    points = load_point_cloud(pc_file)
    print(f"   点云大小: {points.shape}")
    
    # 查找对应的检测结果
    json_file = os.path.join(results_dir, frame_name.replace('.pcd.bin', '.pcd.json'))
    detection_data = None
    
    if os.path.exists(json_file):
        try:
            detection_data = load_detection_results(json_file)
            print(f"   加载检测结果: {json_file}")
        except Exception as e:
            print(f"   加载检测结果失败: {e}")
    else:
        print(f"   未找到检测结果文件: {json_file}")
    
    # 创建可视化
    vis = visualize_detection_results(points, detection_data or {}, frame_name)
    
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
    
    print("🎯 Open3D 3D 检测结果可视化")
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
