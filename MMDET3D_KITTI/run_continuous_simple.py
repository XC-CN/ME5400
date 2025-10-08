#!/usr/bin/env python3
"""
简单的连续帧推理脚本 - 在 nuScenes mini 数据集上运行 CenterPoint
"""

import os
import glob
import time
import subprocess

def run_continuous_simple():
    """运行简单的连续帧推理"""
    
    # 配置参数
    config_file = "configs/centerpoint/centerpoint_voxel01_second_secfpn_head-circlenms_8xb4-cyclic-20e_nus-3d.py"
    checkpoint_file = "checkpoints/centerpoint_nuscenes.pth"
    data_dir = "/home/zzy/CenterPoint/data/nuScenes/samples/LIDAR_TOP/"
    
    # 获取所有点云文件
    point_cloud_files = sorted(glob.glob(os.path.join(data_dir, "*.pcd.bin")))
    print(f"找到 {len(point_cloud_files)} 个点云文件")
    
    # 选择前5个文件进行连续推理
    selected_files = point_cloud_files[:5]
    print(f"将处理前 {len(selected_files)} 个文件")
    
    for i, pc_file in enumerate(selected_files):
        print(f"\n{'='*60}")
        print(f"处理第 {i+1}/{len(selected_files)} 帧: {os.path.basename(pc_file)}")
        print(f"{'='*60}")
        
        # 构建命令
        cmd = [
            "conda", "run", "-n", "mmdet3d", "python", "demo/pcd_demo.py",
            pc_file,
            config_file,
            checkpoint_file,
            "--show",
            "--print-result"
        ]
        
        try:
            # 运行推理
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=60)
            
            if result.returncode == 0:
                print(f"✅ 第 {i+1} 帧推理成功")
                # 打印检测结果摘要
                if "Detected" in result.stdout:
                    print("检测结果摘要:")
                    lines = result.stdout.split('\n')
                    for line in lines:
                        if "Detected" in line or "bboxes_3d" in line:
                            print(f"  {line}")
            else:
                print(f"❌ 第 {i+1} 帧推理失败")
                print(f"错误信息: {result.stderr}")
                
        except subprocess.TimeoutExpired:
            print(f"⏰ 第 {i+1} 帧推理超时")
        except Exception as e:
            print(f"❌ 第 {i+1} 帧推理出错: {e}")
        
        # 添加延迟，让用户有时间查看可视化结果
        if i < len(selected_files) - 1:  # 不是最后一帧
            print(f"\n等待 5 秒后处理下一帧...")
            time.sleep(5)
    
    print(f"\n🎉 连续推理完成！")

if __name__ == "__main__":
    run_continuous_simple()



