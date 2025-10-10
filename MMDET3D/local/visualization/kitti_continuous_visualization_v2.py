#!/usr/bin/env python3
"""
KITTI PointPillars 连续可视化脚本 - 完全模仿nuScenes版本
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

class KittiContinuousVisualizerInteractive:
    def __init__(self, config_file, checkpoint_file, device='cuda:0', score_thr=0.05):
        self.config_file = config_file
        self.checkpoint_file = checkpoint_file
        self.device = device
        self.score_thr = score_thr
        self.inferencer = None
        self.point_cloud_files = []
        self.current_frame_idx = 0
        
        # 控制标志
        self.flag_pause = False
        self.flag_next = False
        self.flag_prev = False
        self.flag_exit = False
        self.frame_delay = 0.5  # 帧间延迟（秒）
        
        # KITTI类别名称
        self.class_names = ['Pedestrian', 'Cyclist', 'Car']
        
        self._init_inferencer()

    def _init_inferencer(self):
        print("初始化 PointPillars 推理器...")
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
            self.flag_pause = not self.flag_pause
            if self.flag_pause:
                print("播放暂停，按空格继续")
            else:
                print("播放继续，按空格暂停")
        elif key == 262:  # 右箭头 - 下一帧
            self.flag_next = True
            print("手动切换到下一帧")
        elif key == 263:  # 左箭头 - 上一帧
            self.flag_prev = True
            print("手动切换到上一帧")
        elif key == 256:  # ESC - 退出
            self.flag_exit = True
            print("退出可视化")
        return False

    def load_sequence(self, sequence_path: Path, max_frames: int = 0):
        """加载点云序列文件"""
        if not sequence_path.exists():
            print(f"错误: 未找到数据集目录 {sequence_path}")
            return

        # 查找所有.bin文件
        self.point_cloud_files = sorted(sequence_path.glob('*.bin'))
        if len(self.point_cloud_files) == 0:
            print(f"错误: 在 {sequence_path} 中未找到 .bin 文件")
            return

        if max_frames > 0:
            self.point_cloud_files = self.point_cloud_files[:max_frames]
        print(f"找到 {len(self.point_cloud_files)} 个点云文件，将处理前 {len(self.point_cloud_files)} 个")


    def run_single_frame(self, bin_file):
        """运行单帧推理和可视化（使用官方inferencer）"""
        print(f"\n处理帧: {bin_file.name} ({self.current_frame_idx + 1}/{len(self.point_cloud_files)})")
        
        # 使用官方inferencer进行推理和可视化
        result = self.inferencer(
            inputs=dict(points=str(bin_file)),
            show=True,
            wait_time=0,  # 不等待，立即返回
            pred_score_thr=self.score_thr,
            no_save_vis=True,  # 不保存可视化结果
            no_save_pred=True  # 不保存预测结果
        )
        
        return result

    def run(self, sequence_path: Path, max_frames: int = 0, interval_s: float = 0.5):
        """运行连续可视化动画"""
        self.load_sequence(sequence_path, max_frames)
        if not self.point_cloud_files:
            return

        print(f"\n开始连续可视化 (KITTI PointPillars)...")
        print("控制说明:")
        print("  空格键: 暂停/继续")
        print("  右箭头: 下一帧")
        print("  左箭头: 上一帧")
        print("  ESC: 退出")
        print("  鼠标滚轮: 缩放")
        print("  鼠标左键拖拽: 旋转视角")
        print("  鼠标右键拖拽: 平移视角")
        print(f"  总共 {len(self.point_cloud_files)} 帧")

        # 显示第一帧
        self.run_single_frame(self.point_cloud_files[self.current_frame_idx])

        # 主循环
        while not self.flag_exit:
            if not self.flag_pause:
                # 自动播放
                time.sleep(interval_s)
                self.current_frame_idx = (self.current_frame_idx + 1) % len(self.point_cloud_files)
                self.run_single_frame(self.point_cloud_files[self.current_frame_idx])
            elif self.flag_next:
                # 手动下一帧
                self.current_frame_idx = (self.current_frame_idx + 1) % len(self.point_cloud_files)
                self.run_single_frame(self.point_cloud_files[self.current_frame_idx])
                self.flag_next = False
            elif self.flag_prev:
                # 手动上一帧
                self.current_frame_idx = (self.current_frame_idx - 1 + len(self.point_cloud_files)) % len(self.point_cloud_files)
                self.run_single_frame(self.point_cloud_files[self.current_frame_idx])
                self.flag_prev = False
            
            # 短暂休眠避免CPU占用过高
            time.sleep(0.01)

        print("可视化结束")

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='KITTI PointPillars 连续可视化')
    parser.add_argument('--input', required=True, help='输入点云目录')
    parser.add_argument('--config', 
                       default='configs/pointpillars/pointpillars_hv_secfpn_8xb6-160e_kitti-3d-3class.py',
                       help='模型配置文件')
    parser.add_argument('--checkpoint', 
                       default='checkpoints/pointpillars_hv_secfpn_6x8_160e_kitti-3d-3class.pth',
                       help='模型检查点文件')
    parser.add_argument('--device', default='cuda:0', help='推理设备')
    parser.add_argument('--max-frames', type=int, default=0, help='最大帧数 (0表示全部)')
    parser.add_argument('--score-thr', type=float, default=0.05, help='置信度阈值')
    parser.add_argument('--interval', type=float, default=0.5, help='帧间延迟（秒）')
    
    args = parser.parse_args()
    
    # 创建可视化器
    visualizer = KittiContinuousVisualizerInteractive(
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
