#!/usr/bin/env python3
"""
检查 nuScenes 点云数据格式
"""

import numpy as np
from pathlib import Path

def check_data_format(bin_file):
    """检查数据格式"""
    print(f"检查文件: {bin_file}")
    
    # 读取原始字节
    with open(bin_file, 'rb') as f:
        raw_data = f.read()
    
    print(f"文件大小: {len(raw_data)} 字节")
    print(f"文件大小 / 4 = {len(raw_data) / 4}")
    print(f"文件大小 / 5 = {len(raw_data) / 5}")
    print(f"文件大小 / 6 = {len(raw_data) / 6}")
    
    # 尝试不同的数据类型
    print("\n尝试不同的数据类型:")
    
    # float32, 4维
    try:
        data_4d = np.fromfile(bin_file, dtype=np.float32)
        if len(data_4d) % 4 == 0:
            points_4d = data_4d.reshape(-1, 4)
            print(f"float32, 4维: {points_4d.shape}")
            print(f"  范围: X[{points_4d[:, 0].min():.2f}, {points_4d[:, 0].max():.2f}]")
            print(f"  范围: Y[{points_4d[:, 1].min():.2f}, {points_4d[:, 1].max():.2f}]")
            print(f"  范围: Z[{points_4d[:, 2].min():.2f}, {points_4d[:, 2].max():.2f}]")
            print(f"  范围: I[{points_4d[:, 3].min():.2f}, {points_4d[:, 3].max():.2f}]")
    except Exception as e:
        print(f"float32, 4维 失败: {e}")
    
    # float32, 5维 (nuScenes 标准格式)
    try:
        data_5d = np.fromfile(bin_file, dtype=np.float32)
        if len(data_5d) % 5 == 0:
            points_5d = data_5d.reshape(-1, 5)
            print(f"float32, 5维: {points_5d.shape}")
            print(f"  范围: X[{points_5d[:, 0].min():.2f}, {points_5d[:, 0].max():.2f}]")
            print(f"  范围: Y[{points_5d[:, 1].min():.2f}, {points_5d[:, 1].max():.2f}]")
            print(f"  范围: Z[{points_5d[:, 2].min():.2f}, {points_5d[:, 2].max():.2f}]")
            print(f"  范围: I[{points_5d[:, 3].min():.2f}, {points_5d[:, 3].max():.2f}]")
            print(f"  范围: R[{points_5d[:, 4].min():.2f}, {points_5d[:, 4].max():.2f}]")
    except Exception as e:
        print(f"float32, 5维 失败: {e}")
    
    # uint8 格式
    try:
        data_uint8 = np.fromfile(bin_file, dtype=np.uint8)
        print(f"uint8: {data_uint8.shape}")
        print(f"  范围: [{data_uint8.min()}, {data_uint8.max()}]")
    except Exception as e:
        print(f"uint8 失败: {e}")
    
    # 检查前几个字节的十六进制
    print(f"\n前32个字节的十六进制:")
    print(' '.join(f'{b:02x}' for b in raw_data[:32]))
    
    # 检查是否有明显的模式
    print(f"\n检查数据模式:")
    unique_values = np.unique(data_uint8[:1000])  # 检查前1000个字节
    print(f"前1000个字节的唯一值数量: {len(unique_values)}")
    print(f"唯一值: {sorted(unique_values)[:20]}...")

if __name__ == "__main__":
    # 检查第一个点云文件
    pc_file = "/home/zzy/CenterPoint/data/nuScenes/samples/LIDAR_TOP/n008-2018-08-01-15-16-36-0400__LIDAR_TOP__1533151603547590.pcd.bin"
    
    if Path(pc_file).exists():
        check_data_format(pc_file)
    else:
        print(f"文件不存在: {pc_file}")
        
        # 尝试找到其他文件
        import glob
        data_dir = "/home/zzy/CenterPoint/data/nuScenes/samples/LIDAR_TOP/"
        files = glob.glob(os.path.join(data_dir, "*.pcd.bin"))
        if files:
            print(f"找到 {len(files)} 个点云文件，检查第一个:")
            check_data_format(files[0])
        else:
            print("没有找到点云文件")



