#!/usr/bin/env python3
"""
可视化推理结果脚本 - 连续播放所有帧的检测结果
"""

import os
import glob
import json
import time
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation

def load_detection_results(results_dir):
    """加载检测结果"""
    pred_files = sorted(glob.glob(os.path.join(results_dir, "*.pkl")))
    print(f"找到 {len(pred_files)} 个预测结果文件")
    
    results = []
    for pred_file in pred_files:
        try:
            # 这里需要根据实际保存的格式来加载
            # 假设保存的是 pickle 文件
            import pickle
            with open(pred_file, 'rb') as f:
                pred_data = pickle.load(f)
            results.append(pred_data)
        except Exception as e:
            print(f"加载 {pred_file} 失败: {e}")
            continue
    
    return results

def create_3d_visualization(points, bboxes, labels, scores, frame_id):
    """创建3D可视化"""
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # 绘制点云
    if len(points) > 0:
        # 随机采样点云以提高性能
        if len(points) > 10000:
            indices = np.random.choice(len(points), 10000, replace=False)
            points_sample = points[indices]
        else:
            points_sample = points
        
        ax.scatter(points_sample[:, 0], points_sample[:, 1], points_sample[:, 2], 
                  c=points_sample[:, 2], cmap='viridis', s=0.5, alpha=0.6)
    
    # 绘制3D边界框
    if len(bboxes) > 0:
        for i, bbox in enumerate(bboxes):
            if len(bbox) >= 7:  # [x, y, z, dx, dy, dz, yaw]
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
                
                # 绘制边界框
                # 定义12条边
                edges = [
                    [0, 1], [1, 2], [2, 3], [3, 0],  # 底面
                    [4, 5], [5, 6], [6, 7], [7, 4],  # 顶面
                    [0, 4], [1, 5], [2, 6], [3, 7]   # 连接边
                ]
                
                for edge in edges:
                    points_edge = corners[edge]
                    ax.plot3D(points_edge[:, 0], points_edge[:, 1], points_edge[:, 2], 
                             'r-', linewidth=2)
                
                # 添加标签和分数
                if i < len(labels) and i < len(scores):
                    ax.text(x, y, z + dz/2 + 0.5, 
                           f'{labels[i]}: {scores[i]:.2f}', 
                           fontsize=8, color='red')
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title(f'Frame {frame_id} - 3D Object Detection Results')
    
    # 设置坐标轴范围
    if len(points) > 0:
        ax.set_xlim([points[:, 0].min()-10, points[:, 0].max()+10])
        ax.set_ylim([points[:, 1].min()-10, points[:, 1].max()+10])
        ax.set_zlim([points[:, 2].min()-5, points[:, 2].max()+5])
    
    plt.tight_layout()
    return fig

def create_continuous_visualization():
    """创建连续可视化"""
    results_dir = "all_frames_results"
    
    # 检查结果目录
    if not os.path.exists(results_dir):
        print(f"结果目录 {results_dir} 不存在！")
        return
    
    # 加载处理摘要
    summary_file = os.path.join(results_dir, "processing_summary.json")
    if os.path.exists(summary_file):
        with open(summary_file, 'r', encoding='utf-8') as f:
            summary = json.load(f)
        print(f"加载处理摘要: {summary['total_frames']} 帧，成功率 {summary['success_rate']:.1f}%")
    
    # 尝试加载检测结果
    try:
        results = load_detection_results(results_dir)
        if not results:
            print("没有找到检测结果文件，尝试从原始点云重新生成...")
            return create_visualization_from_original()
    except Exception as e:
        print(f"加载检测结果失败: {e}")
        return create_visualization_from_original()

def create_visualization_from_original():
    """从原始点云创建可视化"""
    print("从原始点云数据创建可视化...")
    
    # 获取点云文件
    data_dir = "/home/zzy/CenterPoint/data/nuScenes/samples/LIDAR_TOP/"
    point_cloud_files = sorted(glob.glob(os.path.join(data_dir, "*.pcd.bin")))
    
    if not point_cloud_files:
        print("没有找到点云文件！")
        return
    
    print(f"找到 {len(point_cloud_files)} 个点云文件")
    
    # 选择前20帧进行演示
    selected_files = point_cloud_files[:20]
    
    def load_point_cloud(bin_file):
        """加载点云数据"""
        points = np.fromfile(bin_file, dtype=np.float32)
        if points.shape[0] % 4 != 0:
            points = points[:points.shape[0] - (points.shape[0] % 4)]
        return points.reshape(-1, 4)
    
    def animate_frame(frame_idx):
        """动画帧函数"""
        if frame_idx >= len(selected_files):
            return
        
        pc_file = selected_files[frame_idx]
        points = load_point_cloud(pc_file)
        
        # 创建3D图
        fig = plt.figure(figsize=(12, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        # 绘制点云
        if len(points) > 0:
            # 随机采样以提高性能
            if len(points) > 5000:
                indices = np.random.choice(len(points), 5000, replace=False)
                points_sample = points[indices]
            else:
                points_sample = points
            
            ax.scatter(points_sample[:, 0], points_sample[:, 1], points_sample[:, 2], 
                      c=points_sample[:, 2], cmap='viridis', s=1, alpha=0.6)
        
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.set_title(f'Frame {frame_idx + 1}/{len(selected_files)} - {os.path.basename(pc_file)}')
        
        # 设置坐标轴范围
        if len(points) > 0:
            ax.set_xlim([points[:, 0].min()-10, points[:, 0].max()+10])
            ax.set_ylim([points[:, 1].min()-10, points[:, 1].max()+10])
            ax.set_zlim([points[:, 2].min()-5, points[:, 2].max()+5])
        
        plt.tight_layout()
        return fig
    
    # 创建动画
    print("创建连续可视化动画...")
    print("按 Ctrl+C 停止动画")
    
    try:
        for i in range(len(selected_files)):
            fig = animate_frame(i)
            plt.show(block=False)
            plt.pause(2)  # 每帧显示2秒
            plt.close(fig)
    except KeyboardInterrupt:
        print("\n动画已停止")
    
    print("可视化完成！")

def create_static_visualization():
    """创建静态可视化 - 显示几帧关键帧"""
    print("创建静态可视化...")
    
    data_dir = "/home/zzy/CenterPoint/data/nuScenes/samples/LIDAR_TOP/"
    point_cloud_files = sorted(glob.glob(os.path.join(data_dir, "*.pcd.bin")))
    
    # 选择几个关键帧
    key_frames = [0, 50, 100, 150, 200, 250, 300, 350, 400]
    key_frames = [f for f in key_frames if f < len(point_cloud_files)]
    
    def load_point_cloud(bin_file):
        """加载点云数据"""
        points = np.fromfile(bin_file, dtype=np.float32)
        if points.shape[0] % 4 != 0:
            points = points[:points.shape[0] - (points.shape[0] % 4)]
        return points.reshape(-1, 4)
    
    # 创建子图
    fig, axes = plt.subplots(2, 3, figsize=(18, 12), subplot_kw={'projection': '3d'})
    axes = axes.flatten()
    
    for i, frame_idx in enumerate(key_frames[:6]):  # 最多显示6帧
        if i >= len(axes):
            break
            
        pc_file = point_cloud_files[frame_idx]
        points = load_point_cloud(pc_file)
        
        ax = axes[i]
        
        # 绘制点云
        if len(points) > 0:
            # 随机采样
            if len(points) > 3000:
                indices = np.random.choice(len(points), 3000, replace=False)
                points_sample = points[indices]
            else:
                points_sample = points
            
            ax.scatter(points_sample[:, 0], points_sample[:, 1], points_sample[:, 2], 
                      c=points_sample[:, 2], cmap='viridis', s=1, alpha=0.6)
        
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.set_title(f'Frame {frame_idx + 1}')
        
        # 设置坐标轴范围
        if len(points) > 0:
            ax.set_xlim([points[:, 0].min()-10, points[:, 0].max()+10])
            ax.set_ylim([points[:, 1].min()-10, points[:, 1].max()+10])
            ax.set_zlim([points[:, 2].min()-5, points[:, 2].max()+5])
    
    # 隐藏多余的子图
    for i in range(len(key_frames), len(axes)):
        axes[i].set_visible(False)
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    print("选择可视化模式:")
    print("1. 连续动画可视化 (推荐)")
    print("2. 静态多帧可视化")
    
    choice = input("请输入选择 (1 或 2): ").strip()
    
    if choice == "1":
        create_visualization_from_original()
    elif choice == "2":
        create_static_visualization()
    else:
        print("无效选择，使用默认的连续动画可视化")
        create_visualization_from_original()



