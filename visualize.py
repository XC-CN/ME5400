#!/usr/bin/env python3
"""
MCTrack 可视化启动器
直接运行即可查看跟踪结果的可视化效果
"""
import sys
import os
import cv2
import numpy as np
import time
from pathlib import Path
from collections import deque, defaultdict

# 添加visual目录到路径
sys.path.append(str(Path(__file__).parent / "visual"))
from realtime_visualizer import RealTimeVisualizer

def load_kitti_tracking_data(file_path):
    """加载KITTI格式的跟踪数据"""
    tracking_data = {}
    
    with open(file_path, 'r') as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) >= 18:
                frame_id = int(parts[0])
                track_id = int(parts[1])
                obj_type = parts[2]
                
                # 3D bounding box信息 (camera坐标系)
                h, w, l = float(parts[10]), float(parts[11]), float(parts[12])
                x, y, z = float(parts[13]), float(parts[14]), float(parts[15])
                ry = float(parts[16])
                
                if frame_id not in tracking_data:
                    tracking_data[frame_id] = []
                
                tracking_data[frame_id].append({
                    'track_id': track_id,
                    'obj_type': obj_type,
                    'bbox_3d': {
                        'location': [x, y, z],
                        'dimensions': [l, w, h],
                        'rotation_y': ry
                    },
                    'score': float(parts[17]) if len(parts) > 17 else 1.0
                })
    
    return tracking_data

def find_latest_result_file():
    """查找最新的跟踪结果文件"""
    result_dir = Path("results")
    if not result_dir.exists():
        return None
    
    # 查找所有KITTI格式的跟踪结果文件
    txt_files = list(result_dir.rglob("data/*.txt"))
    
    if not txt_files:
        return None
    
    # 返回最新的文件
    return max(txt_files, key=lambda x: x.stat().st_mtime)

def main():
    print("=" * 60)
    print("MCTrack 实时可视化工具")
    print("=" * 60)
    print("功能: 显示车辆跟踪轨迹和实时性能指标")
    print("控制: 'q'退出, '空格'暂停/继续, 'r'重置")
    print("=" * 60)
    
    # 查找跟踪结果文件
    result_file = find_latest_result_file()
    if not result_file:
        print("未找到跟踪结果文件！")
        print("请先运行MCTrack生成结果:")
        print("   python main.py --dataset kitti -e -p 1")
        input("按Enter键退出...")
        return
    
    print(f"找到跟踪结果: {result_file}")
    
    # 加载跟踪数据
    print("正在加载跟踪数据...")
    tracking_data = load_kitti_tracking_data(result_file)
    frames = sorted(tracking_data.keys())
    
    total_objects = sum(len(tracking_data[f]) for f in frames)
    scene_name = result_file.stem
    
    print(f"数据统计:")
    print(f"   - 场景: {scene_name}")
    print(f"   - 总帧数: {len(frames)}")
    print(f"   - 总目标数: {total_objects}")
    print(f"   - 平均每帧目标数: {total_objects/len(frames):.1f}")
    
    # 创建可视化器
    print("正在启动可视化...")
    visualizer = RealTimeVisualizer(
        window_name="MCTrack - 实时车辆跟踪可视化", 
        save_video=False
    )
    
    print("\n可视化已启动!")
    print("=" * 60)
    print("控制说明:")
    print("  'q' - 退出程序")
    print("  '空格' - 暂停/继续播放") 
    print("  'r' - 重置并重新播放")
    print("  'ESC' - 退出程序")
    print("  '+/-' - 调整播放速度")
    print("=" * 60)
    
    # 可视化循环
    paused = False
    current_frame = 0
    play_speed = 150  # ms between frames
    
    try:
        while True:
            if not paused:
                if current_frame >= len(frames):
                    print("播放完成，重新开始...")
                    current_frame = 0
                    visualizer.track_histories.clear()
                    visualizer.track_speeds.clear()
                
                frame_id = frames[current_frame]
                frame_data = tracking_data[frame_id]
                
                # 更新可视化
                visualizer.update_kitti_frame(frame_data, scene_name, frame_id)
                
                current_frame += 1
            
            # 处理按键
            key = cv2.waitKey(play_speed if not paused else 1) & 0xFF
            
            if key == ord('q') or key == 27:  # 'q' 或 ESC
                print("用户退出")
                break
            elif key == ord(' '):  # 空格键暂停/继续
                paused = not paused
                status = "已暂停" if paused else "继续播放"
                print(status)
            elif key == ord('r'):  # 重置
                current_frame = 0
                visualizer.track_histories.clear() 
                visualizer.track_speeds.clear()
                paused = False
                print("已重置")
            elif key == ord('=') or key == ord('+'):  # 加速
                play_speed = max(50, play_speed - 25)
                print(f"播放速度: {1000//play_speed} FPS")
            elif key == ord('-') or key == ord('_'):  # 减速  
                play_speed = min(500, play_speed + 25)
                print(f"播放速度: {1000//play_speed} FPS")
    
    except KeyboardInterrupt:
        print("\n程序被中断")
    
    finally:
        print("正在清理资源...")
        visualizer.cleanup()
        print("可视化已结束")
        
        # 保持窗口开启等待用户
        print("\n按Enter键退出...")
        input()

if __name__ == "__main__":
    main()