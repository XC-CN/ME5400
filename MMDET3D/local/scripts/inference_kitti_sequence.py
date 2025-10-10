#!/usr/bin/env python3
"""
使用 CenterPoint 对 KITTI 原始序列数据进行推理
"""

import os
import sys
import numpy as np
import torch
from pathlib import Path
import argparse
from mmdet3d.apis import init_model, inference_detector
from mmdet3d.structures import Det3DDataSample
import open3d as o3d

def load_kitti_point_cloud(bin_file):
    """加载 KITTI 点云数据"""
    points = np.fromfile(bin_file, dtype=np.float32).reshape(-1, 4)
    return points

def visualize_results(points, results, output_path=None):
    """可视化检测结果"""
    # 创建点云
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points[:, :3])
    
    # 设置点云颜色
    colors = np.ones((len(points), 3)) * 0.5  # 灰色
    pcd.colors = o3d.utility.Vector3dVector(colors)
    
    # 创建边界框
    geometries = [pcd]
    
    if hasattr(results, 'pred_instances_3d') and len(results.pred_instances_3d) > 0:
        bboxes_3d = results.pred_instances_3d.bboxes_3d
        scores = results.pred_instances_3d.scores_3d
        labels = results.pred_instances_3d.labels_3d
        
        print(f"检测到 {len(bboxes_3d)} 个目标:")
        
        for i, (bbox, score, label) in enumerate(zip(bboxes_3d, scores, labels)):
            # 创建边界框
            bbox_o3d = o3d.geometry.OrientedBoundingBox()
            bbox_o3d.center = bbox[:3]
            bbox_o3d.extent = bbox[3:6]
            bbox_o3d.R = o3d.geometry.get_rotation_matrix_from_axis_angle([0, 0, bbox[6]])
            
            # 设置颜色（根据类别）
            if label == 0:  # Car
                bbox_o3d.color = [1, 0, 0]  # 红色
            elif label == 1:  # Pedestrian
                bbox_o3d.color = [0, 1, 0]  # 绿色
            elif label == 2:  # Cyclist
                bbox_o3d.color = [0, 0, 1]  # 蓝色
            
            geometries.append(bbox_o3d)
            
            print(f"  目标 {i+1}: 类别={label}, 置信度={score:.3f}, 位置={bbox[:3]}")
    
    # 可视化
    o3d.visualization.draw_geometries(geometries, window_name="CenterPoint 检测结果")
    
    # 保存结果（如果指定了输出路径）
    if output_path:
        o3d.io.write_point_cloud(str(output_path), pcd)
        print(f"结果已保存到: {output_path}")

def inference_kitti_sequence(sequence_path, model, output_dir=None):
    """对 KITTI 序列进行推理"""
    
    sequence_path = Path(sequence_path)
    velodyne_path = sequence_path / "velodyne_points" / "data"
    
    if not velodyne_path.exists():
        print(f"错误: 未找到点云数据目录 {velodyne_path}")
        return
    
    # 获取所有点云文件
    point_cloud_files = sorted(velodyne_path.glob("*.bin"))
    
    if len(point_cloud_files) == 0:
        print(f"错误: 在 {velodyne_path} 中未找到 .bin 文件")
        return
    
    print(f"找到 {len(point_cloud_files)} 个点云文件")
    
    # 创建输出目录
    if output_dir:
        output_dir = Path(output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
    
    # 对每个点云文件进行推理
    for i, pc_file in enumerate(point_cloud_files):
        print(f"\n处理第 {i+1}/{len(point_cloud_files)} 个文件: {pc_file.name}")
        
        try:
            # 加载点云数据
            points = load_kitti_point_cloud(pc_file)
            print(f"点云大小: {points.shape}")
            
            # 推理
            result = inference_detector(model, str(pc_file))
            
            # 可视化结果
            if output_dir:
                output_file = output_dir / f"{pc_file.stem}_result.ply"
                visualize_results(points, result, output_file)
            else:
                visualize_results(points, result)
                
        except Exception as e:
            print(f"处理文件 {pc_file.name} 时出错: {e}")
            continue

def main():
    parser = argparse.ArgumentParser(description='使用 CenterPoint 对 KITTI 序列进行推理')
    parser.add_argument('--sequence-path', 
                       required=True,
                       help='KITTI 序列数据路径')
    parser.add_argument('--config', 
                       default='configs/centerpoint/centerpoint_voxel01_second_secfpn_8xb4-cyclic-20e_nus-3d.py',
                       help='模型配置文件')
    parser.add_argument('--checkpoint', 
                       default='checkpoints/centerpoint_nuscenes.pth',
                       help='模型检查点文件')
    parser.add_argument('--output-dir', 
                       default=None,
                       help='输出目录（可选）')
    parser.add_argument('--device', 
                       default='cuda:0',
                       help='设备')
    
    args = parser.parse_args()
    
    # 检查文件是否存在
    if not os.path.exists(args.config):
        print(f"错误: 配置文件不存在 {args.config}")
        return
    
    if not os.path.exists(args.checkpoint):
        print(f"错误: 检查点文件不存在 {args.checkpoint}")
        return
    
    if not os.path.exists(args.sequence_path):
        print(f"错误: 序列路径不存在 {args.sequence_path}")
        return
    
    print("初始化 CenterPoint 模型...")
    print(f"配置文件: {args.config}")
    print(f"检查点: {args.checkpoint}")
    print(f"设备: {args.device}")
    
    # 初始化模型
    model = init_model(args.config, args.checkpoint, device=args.device)
    
    print("开始推理...")
    inference_kitti_sequence(args.sequence_path, model, args.output_dir)
    
    print("推理完成!")

if __name__ == "__main__":
    main()
