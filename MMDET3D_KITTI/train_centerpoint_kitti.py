#!/usr/bin/env python3
"""
CenterPoint 在 KITTI 数据集上的训练脚本
"""

import os
import sys
import argparse
from pathlib import Path

def main():
    parser = argparse.ArgumentParser(description='训练 CenterPoint 在 KITTI 数据集上')
    parser.add_argument('--config', 
                       default='configs/centerpoint/centerpoint_voxel01_second_secfpn_8xb4-cyclic-20e_kitti-3d-3class.py',
                       help='配置文件路径')
    parser.add_argument('--work-dir', 
                       default='work_dirs/centerpoint_kitti',
                       help='工作目录')
    parser.add_argument('--resume', 
                       action='store_true',
                       help='从检查点恢复训练')
    parser.add_argument('--checkpoint', 
                       default=None,
                       help='检查点文件路径')
    parser.add_argument('--gpu-ids', 
                       type=int, 
                       nargs='+',
                       default=[0],
                       help='GPU ID 列表')
    
    args = parser.parse_args()
    
    # 检查配置文件是否存在
    if not os.path.exists(args.config):
        print(f"错误: 配置文件不存在 {args.config}")
        return
    
    # 创建工作目录
    work_dir = Path(args.work_dir)
    work_dir.mkdir(parents=True, exist_ok=True)
    
    # 构建训练命令
    cmd = [
        'python', 'tools/train.py',
        args.config,
        '--work-dir', str(work_dir),
        '--gpu-ids', ','.join(map(str, args.gpu_ids))
    ]
    
    if args.resume and args.checkpoint:
        cmd.extend(['--resume', args.checkpoint])
    elif args.checkpoint:
        cmd.extend(['--load-from', args.checkpoint])
    
    print("开始训练 CenterPoint 在 KITTI 数据集上...")
    print(f"配置文件: {args.config}")
    print(f"工作目录: {args.work_dir}")
    print(f"GPU IDs: {args.gpu_ids}")
    print(f"训练命令: {' '.join(cmd)}")
    
    # 执行训练
    os.system(' '.join(cmd))

if __name__ == "__main__":
    main()
