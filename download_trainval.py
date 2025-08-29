#!/usr/bin/env python3
"""
nuScenes v1.0-trainval 数据集下载脚本
下载100个场景的数据用于MCTrack训练
"""

import os
import requests
import json
from tqdm import tqdm
import subprocess
import sys

def download_nuscenes_trainval():
    """下载 nuScenes v1.0-trainval 数据集"""
    
    print("开始下载 nuScenes v1.0-trainval 数据集...")
    
    # 目标目录
    data_root = "data/nuScenes/datasets"
    os.makedirs(data_root, exist_ok=True)
    
    # nuScenes 官方下载URLs - 这些需要从官方网站获取
    urls = {
        # 核心数据集组件
        "v1.0-trainval_meta.tgz": "https://www.nuscenes.org/data/v1.0-trainval_meta.tgz",  
        "v1.0-trainval01_blobs.tgz": "https://www.nuscenes.org/data/v1.0-trainval01_blobs.tgz",
        "v1.0-trainval02_blobs.tgz": "https://www.nuscenes.org/data/v1.0-trainval02_blobs.tgz", 
        "v1.0-trainval03_blobs.tgz": "https://www.nuscenes.org/data/v1.0-trainval03_blobs.tgz",
        "v1.0-trainval04_blobs.tgz": "https://www.nuscenes.org/data/v1.0-trainval04_blobs.tgz",
        "v1.0-trainval05_blobs.tgz": "https://www.nuscenes.org/data/v1.0-trainval05_blobs.tgz",
        "v1.0-trainval06_blobs.tgz": "https://www.nuscenes.org/data/v1.0-trainval06_blobs.tgz",
        # 地图数据
        "nuScenes-map-expansion-v1.3.zip": "https://www.nuscenes.org/data/nuScenes-map-expansion-v1.3.zip"
    }
    
    # 检查是否可以访问官方下载
    print("注意: nuScenes需要注册账号并同意许可协议才能下载")
    print("请访问 https://www.nuscenes.org/nuscenes 注册并获取下载链接")
    
    # 尝试使用wget下载（如果可用）
    try:
        # 首先下载元数据
        print("\n1. 下载元数据...")
        meta_file = "v1.0-trainval_meta.tgz"
        if not os.path.exists(os.path.join(data_root, meta_file)):
            print(f"请手动下载 {meta_file} 到 {data_root}")
        
        # 下载前几个blob文件（100个场景大约需要前3-4个文件）
        print("\n2. 下载数据块文件...")
        blob_files = ["v1.0-trainval01_blobs.tgz", "v1.0-trainval02_blobs.tgz", "v1.0-trainval03_blobs.tgz"]
        
        for blob_file in blob_files:
            blob_path = os.path.join(data_root, blob_file)
            if not os.path.exists(blob_path):
                print(f"请手动下载 {blob_file} 到 {data_root}")
        
        # 下载地图文件
        print("\n3. 下载地图数据...")
        map_file = "nuScenes-map-expansion-v1.3.zip"
        map_path = os.path.join(data_root, map_file)
        if not os.path.exists(map_path):
            print(f"请手动下载 {map_file} 到 {data_root}")
            
        print("\n由于nuScenes需要账号验证，请按以下步骤手动下载：")
        print("1. 访问 https://www.nuscenes.org/nuscenes")
        print("2. 注册账号并同意许可协议")
        print("3. 下载以下文件到", data_root)
        print("   - v1.0-trainval_meta.tgz")
        print("   - v1.0-trainval01_blobs.tgz")
        print("   - v1.0-trainval02_blobs.tgz") 
        print("   - v1.0-trainval03_blobs.tgz")
        print("   - nuScenes-map-expansion-v1.3.zip")
        print("4. 下载完成后运行: python download_trainval.py --extract")
        
    except Exception as e:
        print(f"自动下载失败: {e}")
        return False
    
    return True

def extract_data():
    """解压下载的数据文件"""
    print("开始解压数据文件...")
    
    data_root = "data/nuScenes/datasets"
    
    # 解压文件列表
    archives = [
        "v1.0-trainval_meta.tgz",
        "v1.0-trainval01_blobs.tgz", 
        "v1.0-trainval02_blobs.tgz",
        "v1.0-trainval03_blobs.tgz",
        "nuScenes-map-expansion-v1.3.zip"
    ]
    
    for archive in archives:
        archive_path = os.path.join(data_root, archive)
        if os.path.exists(archive_path):
            print(f"解压 {archive}...")
            try:
                if archive.endswith('.tgz'):
                    # 使用tar解压
                    subprocess.run(['tar', '-xzf', archive_path, '-C', data_root], check=True)
                elif archive.endswith('.zip'):
                    # 使用unzip解压
                    subprocess.run(['unzip', '-o', archive_path, '-d', data_root], check=True)
                print(f"✓ {archive} 解压完成")
            except subprocess.CalledProcessError as e:
                print(f"✗ {archive} 解压失败: {e}")
        else:
            print(f"✗ 找不到文件: {archive_path}")
    
    print("数据解压完成！")

def filter_to_100_scenes():
    """将数据集限制为100个场景"""
    print("正在筛选前100个场景...")
    
    from nuscenes.utils import splits
    
    # 获取训练集场景
    splits_scenes = splits.create_splits_scenes()
    train_scenes = splits_scenes['train'][:100]  # 取前100个训练场景
    
    print(f"将使用前100个训练场景: {len(train_scenes)} 个")
    
    # 这里可以进一步过滤数据文件，只保留前100个场景相关的数据
    # 但这需要更复杂的逻辑来映射场景到具体的数据文件
    
    return train_scenes

if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--extract":
        extract_data()
    elif len(sys.argv) > 1 and sys.argv[1] == "--filter":
        filter_to_100_scenes()
    else:
        download_nuscenes_trainval()