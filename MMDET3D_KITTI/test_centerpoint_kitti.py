#!/usr/bin/env python3
"""
CenterPoint 在 KITTI 数据集上的测试脚本
"""

import os
import sys
import argparse
from pathlib import Path

def main():
    parser = argparse.ArgumentParser(description='测试 CenterPoint 在 KITTI 数据集上')
    parser.add_argument('--config', 
                       default='configs/centerpoint/centerpoint_voxel01_second_secfpn_8xb4-cyclic-20e_kitti-3d-3class.py',
                       help='配置文件路径')
    parser.add_argument('--checkpoint', 
                       required=True,
                       help='检查点文件路径')
    parser.add_argument('--work-dir', 
                       default='work_dirs/centerpoint_kitti',
                       help='工作目录')
    parser.add_argument('--gpu-ids', 
                       type=int, 
                       nargs='+',
                       default=[0],
                       help='GPU ID 列表')
    parser.add_argument('--show', 
                       action='store_true',
                       help='显示结果')
    parser.add_argument('--show-dir', 
                       default='work_dirs/centerpoint_kitti/vis',
                       help='可视化结果保存目录')
    
    args = parser.parse_args()
    
    # 检查配置文件是否存在
    if not os.path.exists(args.config):
        print(f"错误: 配置文件不存在 {args.config}")
        return
    
    # 检查检查点文件是否存在
    if not os.path.exists(args.checkpoint):
        print(f"错误: 检查点文件不存在 {args.checkpoint}")
        return
    
    # 构建测试命令
    cmd = [
        'python', 'tools/test.py',
        args.config,
        args.checkpoint,
        '--work-dir', args.work_dir,
        '--gpu-ids', ','.join(map(str, args.gpu_ids))
    ]
    
    if args.show:
        cmd.extend(['--show', '--show-dir', args.show_dir])
    
    print("开始测试 CenterPoint 在 KITTI 数据集上...")
    print(f"配置文件: {args.config}")
    print(f"检查点: {args.checkpoint}")
    print(f"工作目录: {args.work_dir}")
    print(f"GPU IDs: {args.gpu_ids}")
    print(f"测试命令: {' '.join(cmd)}")
    
    # 执行测试
    os.system(' '.join(cmd))

if __name__ == "__main__":
    main()
