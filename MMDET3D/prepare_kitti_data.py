#!/usr/bin/env python3
"""
KITTI 数据集准备脚本
请先下载 KITTI 数据集并解压到 data/kitti/ 目录下
"""

import os
import sys
import shutil
from pathlib import Path

def prepare_kitti_data():
    """准备 KITTI 数据集"""
    
    # 设置路径
    data_root = Path("data/kitti")
    kitti_root = data_root / "raw"
    
    print("准备 KITTI 数据集...")
    print(f"数据根目录: {data_root}")
    print(f"KITTI 原始数据目录: {kitti_root}")
    
    # 检查是否存在 KITTI 数据
    if not kitti_root.exists():
        print(f"错误: 未找到 KITTI 数据目录 {kitti_root}")
        print("请先下载 KITTI 数据集并解压到该目录")
        print("下载地址: http://www.cvlibs.net/datasets/kitti/eval_object.php?obj_benchmark=3d")
        return False
    
    # 创建必要的目录结构
    training_dir = data_root / "training"
    velodyne_dir = training_dir / "velodyne_reduced"
    velodyne_dir.mkdir(parents=True, exist_ok=True)
    
    # 复制点云数据
    source_velodyne = kitti_root / "velodyne" / "training" / "velodyne"
    if source_velodyne.exists():
        print("复制点云数据...")
        for bin_file in source_velodyne.glob("*.bin"):
            shutil.copy2(bin_file, velodyne_dir)
        print(f"已复制 {len(list(velodyne_dir.glob('*.bin')))} 个点云文件")
    else:
        print(f"警告: 未找到点云数据目录 {source_velodyne}")
    
    # 复制标签数据
    source_labels = kitti_root / "label_2" / "training" / "label_2"
    target_labels = training_dir / "label_2"
    if source_labels.exists():
        print("复制标签数据...")
        shutil.copytree(source_labels, target_labels, dirs_exist_ok=True)
        print(f"已复制标签数据到 {target_labels}")
    else:
        print(f"警告: 未找到标签数据目录 {source_labels}")
    
    # 复制图像数据（如果需要）
    source_images = kitti_root / "image_2" / "training" / "image_2"
    target_images = training_dir / "image_2"
    if source_images.exists():
        print("复制图像数据...")
        shutil.copytree(source_images, target_images, dirs_exist_ok=True)
        print(f"已复制图像数据到 {target_images}")
    else:
        print(f"警告: 未找到图像数据目录 {source_images}")
    
    print("KITTI 数据集准备完成！")
    print("现在可以运行以下命令来创建数据索引文件：")
    print("python tools/create_data.py kitti --root-path data/kitti --out-dir data/kitti --extra-tag kitti")
    
    return True

if __name__ == "__main__":
    prepare_kitti_data()
