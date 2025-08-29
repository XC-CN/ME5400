#!/usr/bin/env python3
"""
nuScenes MCTrack结果可视化工具
显示跟踪结果的2D俯视图
"""

import json
import cv2
import numpy as np
import time
import os
from collections import defaultdict, deque
import argparse

class NuScenesVisualizer:
    def __init__(self, result_file, base_version_file=None):
        self.result_file = result_file
        self.base_version_file = base_version_file
        
        # 可视化参数
        self.canvas_size = (1200, 800)
        self.scale = 10  # 米到像素的缩放
        self.origin = (600, 400)  # 画布中心
        
        # 颜色定义
        self.colors = {
            'car': (0, 255, 0),      # 绿色
            'truck': (255, 0, 0),    # 蓝色  
            'bus': (0, 0, 255),      # 红色
            'pedestrian': (255, 255, 0),  # 青色
            'motorcycle': (255, 0, 255),  # 紫色
            'bicycle': (0, 255, 255),     # 黄色
            'trailer': (128, 128, 128),   # 灰色
        }
        
        # 跟踪历史
        self.track_histories = defaultdict(lambda: deque(maxlen=30))
        self.track_colors = {}
        
        # 加载数据
        self.load_results()
        if base_version_file:
            self.load_base_version_data()
    
    def load_results(self):
        """加载MCTrack跟踪结果"""
        print(f"加载跟踪结果: {self.result_file}")
        with open(self.result_file, 'r') as f:
            data = json.load(f)
        
        self.results = data['results']
        self.sample_tokens = list(self.results.keys())
        print(f"加载了 {len(self.sample_tokens)} 个样本的跟踪结果")
    
    def load_base_version_data(self):
        """加载BaseVersion数据以获取ego pose信息"""
        print(f"加载BaseVersion数据: {self.base_version_file}")
        with open(self.base_version_file, 'r') as f:
            self.base_data = json.load(f)
    
    def get_track_color(self, track_id):
        """为轨迹ID分配颜色"""
        if track_id not in self.track_colors:
            # 使用轨迹ID的hash生成一致的颜色
            seed = hash(str(track_id)) % 2147483647
            np.random.seed(seed)
            self.track_colors[track_id] = tuple(np.random.randint(50, 255, 3).tolist())
        return self.track_colors[track_id]
    
    def world_to_screen(self, x, y):
        """世界坐标转屏幕坐标"""
        screen_x = int(self.origin[0] + x * self.scale)
        screen_y = int(self.origin[1] - y * self.scale)  # y轴翻转
        return screen_x, screen_y
    
    def draw_track(self, canvas, track):
        """绘制单个跟踪目标"""
        # 获取位置
        x, y = track['translation'][0], track['translation'][1]
        screen_x, screen_y = self.world_to_screen(x, y)
        
        # 获取颜色
        category = track['tracking_name']
        track_id = track['tracking_id']
        
        # 类别颜色 vs 轨迹ID颜色
        category_color = self.colors.get(category, (255, 255, 255))
        track_color = self.get_track_color(track_id)
        
        # 绘制边界框
        size = track['size']
        w, l = size[0], size[1]  # width, length
        
        # 简化为圆形表示
        radius = int(max(w, l) * self.scale / 2)
        radius = max(radius, 3)
        
        # 绘制轨迹历史
        track_history = self.track_histories[track_id]
        track_history.append((screen_x, screen_y))
        
        if len(track_history) > 1:
            points = list(track_history)
            for i in range(1, len(points)):
                alpha = i / len(points)  # 透明度渐变
                color = tuple(int(c * alpha) for c in track_color)
                cv2.line(canvas, points[i-1], points[i], color, 2)
        
        # 绘制当前位置
        cv2.circle(canvas, (screen_x, screen_y), radius, track_color, -1)
        cv2.circle(canvas, (screen_x, screen_y), radius, category_color, 2)
        
        # 绘制ID标签
        label = f"{track_id}"
        cv2.putText(canvas, label, (screen_x - 10, screen_y - radius - 5),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # 绘制类别标签
        cv2.putText(canvas, category[:3], (screen_x - 10, screen_y + radius + 15),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, category_color, 1)
    
    def draw_frame(self, sample_idx):
        """绘制单帧"""
        canvas = np.zeros((self.canvas_size[1], self.canvas_size[0], 3), dtype=np.uint8)
        
        # 绘制坐标网格
        self.draw_grid(canvas)
        
        # 获取当前样本的跟踪结果
        sample_token = self.sample_tokens[sample_idx]
        tracks = self.results[sample_token]
        
        # 绘制所有跟踪目标
        for track in tracks:
            self.draw_track(canvas, track)
        
        # 绘制信息面板
        self.draw_info_panel(canvas, sample_idx, len(tracks))
        
        return canvas
    
    def draw_grid(self, canvas):
        """绘制坐标网格"""
        # 绘制网格线（每10米一条线）
        grid_spacing = 10 * self.scale  # 10米
        
        # 垂直线
        for x in range(0, self.canvas_size[0], grid_spacing):
            cv2.line(canvas, (x, 0), (x, self.canvas_size[1]), (50, 50, 50), 1)
        
        # 水平线
        for y in range(0, self.canvas_size[1], grid_spacing):
            cv2.line(canvas, (0, y), (self.canvas_size[0], y), (50, 50, 50), 1)
        
        # 绘制坐标轴
        cv2.line(canvas, (self.origin[0], 0), (self.origin[0], self.canvas_size[1]), (100, 100, 100), 2)
        cv2.line(canvas, (0, self.origin[1]), (self.canvas_size[0], self.origin[1]), (100, 100, 100), 2)
        
        # 坐标轴标签
        cv2.putText(canvas, "X (East)", (self.origin[0] + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        cv2.putText(canvas, "Y (North)", (10, self.origin[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
    
    def draw_info_panel(self, canvas, sample_idx, track_count):
        """绘制信息面板"""
        y_offset = 30
        
        # 基本信息
        info_texts = [
            f"Frame: {sample_idx + 1}/{len(self.sample_tokens)}",
            f"Tracks: {track_count}",
            f"Sample: {self.sample_tokens[sample_idx][:8]}...",
        ]
        
        for i, text in enumerate(info_texts):
            y = y_offset + i * 25
            cv2.putText(canvas, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        
        # 类别图例
        legend_y = 150
        cv2.putText(canvas, "Categories:", (10, legend_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        for i, (category, color) in enumerate(self.colors.items()):
            y = legend_y + 20 + i * 20
            cv2.rectangle(canvas, (10, y-10), (30, y+5), color, -1)
            cv2.putText(canvas, category, (35, y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        
        # 控制提示
        controls = [
            "Controls:",
            "Space: Pause/Resume",
            "q: Quit", 
            "r: Restart",
            "n: Next frame",
            "p: Previous frame"
        ]
        
        control_y = self.canvas_size[1] - len(controls) * 20 - 20
        for i, text in enumerate(controls):
            y = control_y + i * 18
            cv2.putText(canvas, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
    
    def run(self, fps=10, start_frame=0):
        """运行可视化"""
        print("开始可视化播放...")
        print("控制键: 空格=暂停/继续, q=退出, r=重启, n=下一帧, p=上一帧")
        
        frame_idx = start_frame
        paused = False
        
        while True:
            if not paused:
                canvas = self.draw_frame(frame_idx)
                cv2.imshow("MCTrack nuScenes Visualization", canvas)
                
                frame_idx = (frame_idx + 1) % len(self.sample_tokens)
            
            # 处理按键
            key = cv2.waitKey(int(1000 / fps)) & 0xFF
            
            if key == ord('q'):
                break
            elif key == ord(' '):
                paused = not paused
                print("暂停" if paused else "继续")
            elif key == ord('r'):
                frame_idx = 0
                self.track_histories.clear()
                print("重启")
            elif key == ord('n'):
                if paused:
                    frame_idx = min(frame_idx + 1, len(self.sample_tokens) - 1)
                    canvas = self.draw_frame(frame_idx)
                    cv2.imshow("MCTrack nuScenes Visualization", canvas)
            elif key == ord('p'):
                if paused:
                    frame_idx = max(frame_idx - 1, 0)
                    canvas = self.draw_frame(frame_idx)
                    cv2.imshow("MCTrack nuScenes Visualization", canvas)
        
        cv2.destroyAllWindows()
        print("可视化结束")

def main():
    parser = argparse.ArgumentParser(description='MCTrack nuScenes 结果可视化')
    parser.add_argument('--result-file', type=str, 
                       default='results/nuscenes/20250826_161302/results.json',
                       help='跟踪结果文件路径')
    parser.add_argument('--base-version-file', type=str,
                       default='data/base_version/nuscenes/centerpoint/val.json',
                       help='BaseVersion数据文件路径')
    parser.add_argument('--fps', type=int, default=5, help='播放帧率')
    parser.add_argument('--start-frame', type=int, default=0, help='开始帧')
    
    args = parser.parse_args()
    
    # 检查文件是否存在
    if not os.path.exists(args.result_file):
        print(f"错误: 跟踪结果文件不存在: {args.result_file}")
        print("请先运行 MCTrack 生成结果: python main.py --dataset nuscenes -e -p 1")
        return
    
    base_version_file = args.base_version_file if os.path.exists(args.base_version_file) else None
    
    # 创建可视化器
    visualizer = NuScenesVisualizer(args.result_file, base_version_file)
    
    # 运行可视化
    visualizer.run(fps=args.fps, start_frame=args.start_frame)

if __name__ == "__main__":
    main()