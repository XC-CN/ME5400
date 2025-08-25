#!/usr/bin/env python3
"""
MCTrack 一键启动可视化工具
"""
import argparse
import os
import sys
import subprocess
from pathlib import Path

def check_dependencies():
    """检查依赖"""
    required_packages = {
        'cv2': 'opencv-python',
        'numpy': 'numpy', 
        'open3d': 'open3d'
    }
    
    missing = []
    for module, package in required_packages.items():
        try:
            __import__(module)
        except ImportError:
            missing.append(package)
    
    if missing:
        print(f"Missing packages: {missing}")
        print("Install with: pip install " + " ".join(missing))
        return False
    return True

def run_tracking_first():
    """运行MCTrack生成结果"""
    print("Running MCTrack to generate tracking results...")
    
    cmd = [
        sys.executable, "main.py", 
        "--dataset", "kitti", 
        "-e", "-p", "1"
    ]
    
    try:
        result = subprocess.run(cmd, check=True, capture_output=True, text=True)
        print("MCTrack completed successfully")
        return True
    except subprocess.CalledProcessError as e:
        print(f"MCTrack failed: {e}")
        print("Output:", e.stdout)
        print("Error:", e.stderr)
        return False

def find_result_files():
    """查找结果文件"""
    result_dir = Path("results")
    if not result_dir.exists():
        return []
    
    # 查找KITTI格式的跟踪结果文件
    txt_files = list(result_dir.rglob("data/*.txt"))
    if txt_files:
        return txt_files
    
    # 备用：查找JSON文件
    json_files = list(result_dir.rglob("*.json"))
    return json_files

def load_kitti_tracking_data(file_path):
    """加载KITTI格式的跟踪数据"""
    tracking_data = {}
    
    with open(file_path, 'r') as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) >= 18:  # KITTI跟踪格式至少18个字段
                frame_id = int(parts[0])
                track_id = int(parts[1])
                obj_type = parts[2]
                
                # 3D bounding box信息 (camera坐标系)
                h, w, l = float(parts[10]), float(parts[11]), float(parts[12])
                x, y, z = float(parts[13]), float(parts[14]), float(parts[15])
                ry = float(parts[16])  # rotation_y
                
                # 2D bounding box
                bbox_2d = [float(parts[6]), float(parts[7]), float(parts[8]), float(parts[9])]
                
                if frame_id not in tracking_data:
                    tracking_data[frame_id] = []
                
                tracking_data[frame_id].append({
                    'track_id': track_id,
                    'obj_type': obj_type,
                    'bbox_3d': {
                        'location': [x, y, z],  # camera坐标系
                        'dimensions': [l, w, h],  # length, width, height
                        'rotation_y': ry
                    },
                    'bbox_2d': bbox_2d,
                    'score': float(parts[17]) if len(parts) > 17 else 1.0
                })
    
    return tracking_data

def main():
    parser = argparse.ArgumentParser(description="MCTrack Visualization Launcher")
    parser.add_argument("--mode", choices=['realtime', '3d', 'integrated'], 
                       default='realtime', help="Visualization mode")
    parser.add_argument("--auto-run", action='store_true', 
                       help="Automatically run MCTrack if no results found")
    parser.add_argument("--result-path", type=str, 
                       help="Path to specific result file")
    parser.add_argument("--save-video", action='store_true',
                       help="Save visualization as video")
    
    args = parser.parse_args()
    
    print("MCTrack Visualization Launcher")
    print("=" * 50)
    
    # 检查依赖
    if not check_dependencies():
        return
    
    # 检查结果文件
    result_files = find_result_files()
    
    if not result_files and not args.result_path:
        if args.auto_run:
            if not run_tracking_first():
                return
            result_files = find_result_files()
        else:
            print("No tracking results found!")
            print("Options:")
            print("1. Run with --auto-run to generate results automatically")
            print("2. Run MCTrack manually: python main.py --dataset kitti -e -p 1")
            print("3. Specify result file with --result-path")
            return
    
    # 选择结果文件
    if args.result_path:
        result_path = args.result_path
    elif result_files:
        result_path = str(result_files[0])  # 使用第一个找到的文件
    else:
        print("No valid result file found")
        return
    
    print(f"Using result file: {result_path}")
    
    # 启动对应的可视化工具
    try:
        if args.mode == 'realtime':
            print("Starting Real-time Visualizer...")
            from realtime_visualizer import RealTimeVisualizer
            
            # 创建演示
            visualizer = RealTimeVisualizer(save_video=args.save_video)
            
            # 处理KITTI格式数据
            if result_path.endswith('.txt'):
                print(f"Loading KITTI tracking data from: {result_path}")
                scene_name = Path(result_path).stem  # 文件名作为场景名
                
                # 读取KITTI跟踪数据
                tracking_data = load_kitti_tracking_data(result_path)
                frames = sorted(tracking_data.keys())
                
                print(f"Scene: {scene_name}, Frames: {len(frames)}")
                print("Controls: 'q' to quit, 'space' to pause")
                
                import time
                import cv2
                
                print("Starting visualization loop...")
                
                for i, frame_id in enumerate(frames):
                    frame_data = tracking_data[frame_id]
                    
                    print(f"Processing frame {frame_id} ({i+1}/{len(frames)}), {len(frame_data)} objects")
                    
                    visualizer.update_kitti_frame(frame_data, scene_name, frame_id)
                    
                    key = cv2.waitKey(200) & 0xFF  # 更慢一点以便观察
                    if key == ord('q'):
                        print("Quit requested by user")
                        break
                    elif key == ord(' '):
                        print("Paused - press any key to continue")
                        cv2.waitKey(0)
                
                print("Visualization loop completed. Press any key to close...")
                
            else:
                # JSON格式处理
                import json
                with open(result_path, 'r') as f:
                    data = json.load(f)
                
                scene_name = list(data.keys())[0]
                scene_data = data[scene_name]
                frames = sorted(scene_data.keys(), key=lambda x: int(x.split('_')[1]))
                
                print(f"Scene: {scene_name}, Frames: {len(frames)}")
                print("Controls: 'q' to quit, 'space' to pause")
                
                import time
                import cv2
                
                for i, frame_name in enumerate(frames):
                    frame_data = scene_data[frame_name]
                    visualizer.update_frame(frame_data, scene_name, i)
                    
                    key = cv2.waitKey(50) & 0xFF
                    if key == ord('q'):
                        break
                    elif key == ord(' '):
                        cv2.waitKey(0)
            
            visualizer.cleanup()
            
        elif args.mode == '3d':
            print("Starting 3D Visualizer...")
            cmd = [sys.executable, "visual/visualize_3d.py", 
                   "--result_path", result_path]
            subprocess.run(cmd)
            
        elif args.mode == 'integrated':
            print("Integrated mode requires manual integration")
            print("See visual/README.md for integration instructions")
    
    except KeyboardInterrupt:
        print("\nVisualization stopped by user")
    except Exception as e:
        print(f"Error: {e}")
    
    print("Visualization completed")

if __name__ == "__main__":
    main()