#!/usr/bin/env python3
"""
处理全部帧推理脚本 - 在 nuScenes mini 数据集上运行 CenterPoint 处理所有帧
"""

import os
import glob
import time
import subprocess
import json
from datetime import datetime

def run_all_frames():
    """运行全部帧推理"""
    
    # 配置参数
    config_file = "configs/centerpoint/centerpoint_voxel01_second_secfpn_head-circlenms_8xb4-cyclic-20e_nus-3d.py"
    checkpoint_file = "checkpoints/centerpoint_nuscenes.pth"
    data_dir = "/home/zzy/CenterPoint/data/nuScenes/samples/LIDAR_TOP/"
    output_dir = "all_frames_results"
    
    # 创建输出目录
    os.makedirs(output_dir, exist_ok=True)
    
    # 获取所有点云文件
    point_cloud_files = sorted(glob.glob(os.path.join(data_dir, "*.pcd.bin")))
    total_files = len(point_cloud_files)
    print(f"找到 {total_files} 个点云文件")
    
    # 统计信息
    successful_frames = 0
    failed_frames = 0
    results_summary = []
    
    print(f"开始处理全部 {total_files} 帧...")
    start_time = time.time()
    
    for i, pc_file in enumerate(point_cloud_files):
        frame_name = os.path.basename(pc_file)
        print(f"\n处理第 {i+1}/{total_files} 帧: {frame_name}")
        
        # 构建命令 - 不显示可视化窗口，只保存结果
        cmd = [
            "conda", "run", "-n", "mmdet3d", "python", "demo/pcd_demo.py",
            pc_file,
            config_file,
            checkpoint_file,
            "--out-dir", output_dir,
            "--no-save-vis",  # 不保存可视化结果，只保存预测结果
            "--print-result"
        ]
        
        frame_start_time = time.time()
        
        try:
            # 运行推理
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
            frame_time = time.time() - frame_start_time
            
            if result.returncode == 0:
                successful_frames += 1
                print(f"✅ 第 {i+1} 帧推理成功 (耗时: {frame_time:.2f}s)")
                
                # 解析检测结果
                if "bboxes_3d" in result.stdout:
                    # 简单统计检测到的目标数量
                    lines = result.stdout.split('\n')
                    for line in lines:
                        if "bboxes_3d" in line:
                            # 计算检测到的目标数量
                            bbox_count = line.count('[') - 1  # 粗略估算
                            break
                    else:
                        bbox_count = 0
                else:
                    bbox_count = 0
                
                results_summary.append({
                    "frame_id": i,
                    "frame_name": frame_name,
                    "status": "success",
                    "detection_count": bbox_count,
                    "processing_time": frame_time
                })
                
            else:
                failed_frames += 1
                print(f"❌ 第 {i+1} 帧推理失败 (耗时: {frame_time:.2f}s)")
                print(f"错误信息: {result.stderr[:200]}...")
                
                results_summary.append({
                    "frame_id": i,
                    "frame_name": frame_name,
                    "status": "failed",
                    "detection_count": 0,
                    "processing_time": frame_time,
                    "error": result.stderr[:200]
                })
                
        except subprocess.TimeoutExpired:
            failed_frames += 1
            frame_time = time.time() - frame_start_time
            print(f"⏰ 第 {i+1} 帧推理超时 (耗时: {frame_time:.2f}s)")
            
            results_summary.append({
                "frame_id": i,
                "frame_name": frame_name,
                "status": "timeout",
                "detection_count": 0,
                "processing_time": frame_time
            })
            
        except Exception as e:
            failed_frames += 1
            frame_time = time.time() - frame_start_time
            print(f"❌ 第 {i+1} 帧推理出错: {e} (耗时: {frame_time:.2f}s)")
            
            results_summary.append({
                "frame_id": i,
                "frame_name": frame_name,
                "status": "error",
                "detection_count": 0,
                "processing_time": frame_time,
                "error": str(e)
            })
        
        # 每处理10帧显示一次进度
        if (i + 1) % 10 == 0:
            elapsed_time = time.time() - start_time
            avg_time_per_frame = elapsed_time / (i + 1)
            remaining_frames = total_files - (i + 1)
            estimated_remaining_time = remaining_frames * avg_time_per_frame
            
            print(f"\n📊 进度报告:")
            print(f"   已处理: {i+1}/{total_files} 帧")
            print(f"   成功: {successful_frames} 帧")
            print(f"   失败: {failed_frames} 帧")
            print(f"   平均每帧耗时: {avg_time_per_frame:.2f}s")
            print(f"   预计剩余时间: {estimated_remaining_time/60:.1f} 分钟")
            print(f"   总耗时: {elapsed_time/60:.1f} 分钟")
    
    # 最终统计
    total_time = time.time() - start_time
    print(f"\n{'='*60}")
    print(f"🎉 全部帧处理完成！")
    print(f"{'='*60}")
    print(f"总帧数: {total_files}")
    print(f"成功帧数: {successful_frames}")
    print(f"失败帧数: {failed_frames}")
    print(f"成功率: {successful_frames/total_files*100:.1f}%")
    print(f"总耗时: {total_time/60:.1f} 分钟")
    print(f"平均每帧耗时: {total_time/total_files:.2f}s")
    
    # 保存结果摘要
    summary_file = os.path.join(output_dir, "processing_summary.json")
    summary_data = {
        "timestamp": datetime.now().isoformat(),
        "total_frames": total_files,
        "successful_frames": successful_frames,
        "failed_frames": failed_frames,
        "success_rate": successful_frames/total_files*100,
        "total_processing_time": total_time,
        "average_time_per_frame": total_time/total_files,
        "results": results_summary
    }
    
    with open(summary_file, 'w', encoding='utf-8') as f:
        json.dump(summary_data, f, indent=2, ensure_ascii=False)
    
    print(f"\n📁 结果已保存到: {output_dir}")
    print(f"📄 处理摘要已保存到: {summary_file}")
    
    # 检测结果统计
    if successful_frames > 0:
        total_detections = sum(r["detection_count"] for r in results_summary if r["status"] == "success")
        avg_detections = total_detections / successful_frames
        print(f"🔍 检测统计:")
        print(f"   总检测目标数: {total_detections}")
        print(f"   平均每帧检测数: {avg_detections:.1f}")

if __name__ == "__main__":
    run_all_frames()
