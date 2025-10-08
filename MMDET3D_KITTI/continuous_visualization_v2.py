#!/usr/bin/env python3
"""
基于官方demo逻辑的连续可视化脚本
完全复用官方demo的可视化方式
"""

import os
import sys
import time
import numpy as np
import torch
from pathlib import Path
from typing import List, Optional
import open3d as o3d

# 添加 MMDetection3D 路径
sys.path.append('/home/zzy/mmdet3d_centerpoint')
from mmdet3d.apis import LidarDet3DInferencer
from mmengine.registry import init_default_scope
from mmengine.logging import print_log
import logging
import copy

class ContinuousVisualizerV2:
    def __init__(self, config_file, checkpoint_file, device='cuda:0', score_thr=0.3):
        self.config_file = config_file
        self.checkpoint_file = checkpoint_file
        self.device = device
        self.score_thr = score_thr
        self.inferencer = None
        self.point_cloud_files = []
        self.current_frame_idx = 0
        
        # 控制标志
        self.is_playing = True
        self.is_paused = False
        self.auto_play = True
        self.frame_delay = 0.5  # 帧间延迟（秒）
        
        self._init_inferencer()

    def _init_inferencer(self):
        print("初始化 CenterPoint 推理器...")
        init_default_scope('mmdet3d')
        self.inferencer = LidarDet3DInferencer(
            model=self.config_file,
            weights=self.checkpoint_file,
            device=self.device
        )
        print("推理器初始化成功!")

    def _key_callback(self, vis, key, mods):
        """键盘回调函数"""
        if key == 32:  # 空格 - 暂停/继续
            self.visualizer.flag_pause = not self.visualizer.flag_pause
            if self.visualizer.flag_pause:
                print("播放暂停，按空格继续")
            else:
                print("播放继续，按空格暂停")
        elif key == 262:  # 右箭头 - 下一帧
            self.visualizer.flag_next = True
            print("手动切换到下一帧")
        elif key == 263:  # 左箭头 - 上一帧
            self.visualizer.flag_prev = True
            print("手动切换到上一帧")
        elif key == 256:  # ESC - 退出
            self.visualizer.flag_exit = True
            print("退出可视化")
        return False

    def load_nuscenes_point_cloud(self, bin_file):
        """加载 nuScenes 点云数据 (5维: x, y, z, intensity, ring_index)"""
        points = np.fromfile(bin_file, dtype=np.float32)
        if points.shape[0] % 5 != 0:
            points = points[:points.shape[0] - (points.shape[0] % 5)]
        return points.reshape(-1, 5)

    def load_sequence(self, sequence_path: Path, max_frames: int = 0):
        """加载点云序列文件"""
        if not sequence_path.exists():
            print(f"错误: 未找到数据集目录 {sequence_path}")
            return

        self.point_cloud_files = sorted(sequence_path.glob('*.pcd.bin'))
        if len(self.point_cloud_files) == 0:
            print(f"错误: 在 {sequence_path} 中未找到 .pcd.bin 文件")
            return

        if max_frames > 0:
            self.point_cloud_files = self.point_cloud_files[:max_frames]
        print(f"找到 {len(self.point_cloud_files)} 个点云文件，将处理前 {len(self.point_cloud_files)} 个")

    def update_visualization(self):
        """更新可视化（使用官方逻辑）"""
        if not self.point_cloud_files:
            print("没有点云文件可供可视化。")
            return

        # 清除旧的几何体
        self.visualizer.o3d_vis.clear_geometries()

        # 加载当前帧的点云
        pc_file = self.point_cloud_files[self.current_frame_idx]
        print(f"\n处理帧: {pc_file.name} ({self.current_frame_idx + 1}/{len(self.point_cloud_files)})")
        points = self.load_nuscenes_point_cloud(pc_file)

        # 执行推理
        with torch.no_grad():
            result = inference_detector(self.model, points)

        # 使用官方的可视化方法
        # 创建数据样本
        data_sample = Det3DDataSample()
        data_sample.pred_instances_3d = result.pred_instances_3d
        
        # 使用官方的 add_datasample 方法
        self.visualizer.add_datasample(
            'continuous_vis',
            points,
            data_sample,
            show=False,  # 不自动显示，我们手动控制
            vis_task='lidar_det',
            pred_score_thr=self.score_thr
        )

        # 更新渲染
        self.visualizer.o3d_vis.update_renderer()
        self.visualizer.o3d_vis.poll_events()

    def run(self, sequence_path: Path, max_frames: int = 0, interval_s: float = 0.5):
        """运行连续可视化动画"""
        self.load_sequence(sequence_path, max_frames)
        if not self.point_cloud_files:
            return

        print(f"\n开始连续可视化...")
        print("控制说明:")
        print("  空格键: 暂停/继续")
        print("  右箭头: 下一帧")
        print("  左箭头: 上一帧")
        print("  ESC: 退出")
        print(f"  总共 {len(self.point_cloud_files)} 帧")

        # 显示第一帧
        self.update_visualization()

        # 主循环
        while not self.visualizer.flag_exit:
            if not self.visualizer.flag_pause:
                # 自动播放
                time.sleep(interval_s)
                self.current_frame_idx = (self.current_frame_idx + 1) % len(self.point_cloud_files)
                self.update_visualization()
            elif self.visualizer.flag_next:
                # 手动下一帧
                self.current_frame_idx = (self.current_frame_idx + 1) % len(self.point_cloud_files)
                self.update_visualization()
                self.visualizer.flag_next = False
            elif self.visualizer.flag_prev:
                # 手动上一帧
                self.current_frame_idx = (self.current_frame_idx - 1 + len(self.point_cloud_files)) % len(self.point_cloud_files)
                self.update_visualization()
                self.visualizer.flag_prev = False
            
            # 保持窗口响应
            self.visualizer.o3d_vis.poll_events()
            self.visualizer.o3d_vis.update_renderer()

        # 清理
        self.visualizer.o3d_vis.destroy_window()
        self.visualizer.o3d_vis.close()
        print("可视化结束")

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='CenterPoint 连续可视化')
    parser.add_argument('--input', required=True, help='输入点云目录')
    parser.add_argument('--config', 
                       default='configs/centerpoint/centerpoint_voxel01_second_secfpn_head-circlenms_8xb4-cyclic-20e_nus-3d.py',
                       help='模型配置文件')
    parser.add_argument('--checkpoint', 
                       default='checkpoints/centerpoint_nuscenes.pth',
                       help='模型检查点文件')
    parser.add_argument('--device', default='cuda:0', help='推理设备')
    parser.add_argument('--max-frames', type=int, default=0, help='最大帧数 (0表示全部)')
    parser.add_argument('--score-thr', type=float, default=0.3, help='置信度阈值')
    parser.add_argument('--interval', type=float, default=0.5, help='帧间延迟（秒）')
    
    args = parser.parse_args()
    
    # 创建可视化器
    visualizer = ContinuousVisualizerV2(
        config_file=args.config,
        checkpoint_file=args.checkpoint,
        device=args.device,
        score_thr=args.score_thr
    )
    
    # 运行可视化
    visualizer.run(
        sequence_path=Path(args.input),
        max_frames=args.max_frames,
        interval_s=args.interval
    )

if __name__ == "__main__":
    main()
