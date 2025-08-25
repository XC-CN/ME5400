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
    
    json_files = list(result_dir.rglob("*.json"))
    return json_files

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
            print("❌ No tracking results found!")
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
        print("❌ No valid result file found")
        return
    
    print(f"📊 Using result file: {result_path}")
    
    # 启动对应的可视化工具
    try:
        if args.mode == 'realtime':
            print("🎬 Starting Real-time Visualizer...")
            from realtime_visualizer import RealTimeVisualizer
            
            # 创建演示
            visualizer = RealTimeVisualizer(save_video=args.save_video)
            
            import json
            with open(result_path, 'r') as f:
                data = json.load(f)
            
            # 简单播放演示
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
            print("🎮 Starting 3D Visualizer...")
            cmd = [sys.executable, "visual/visualize_3d.py", 
                   "--result_path", result_path]
            subprocess.run(cmd)
            
        elif args.mode == 'integrated':
            print("🔗 Integrated mode requires manual integration")
            print("See visual/README.md for integration instructions")
    
    except KeyboardInterrupt:
        print("\n⏹️  Visualization stopped by user")
    except Exception as e:
        print(f"❌ Error: {e}")
    
    print("✅ Visualization completed")

if __name__ == "__main__":
    main()