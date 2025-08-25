#!/usr/bin/env python3
"""
MCTrack 实时可视化工具
集成到跟踪流程中，显示实时跟踪性能和运动车辆
"""
import cv2
import numpy as np
import time
import threading
from collections import deque, defaultdict
import json
import os

class RealTimeVisualizer:
    def __init__(self, window_name="MCTrack Real-Time Tracking", save_video=False):
        self.window_name = window_name
        self.save_video = save_video
        
        # 可视化参数
        self.canvas_size = (1600, 1200)
        self.scale = 8  # 坐标缩放因子
        self.origin = (800, 600)  # 画布中心点
        
        # 轨迹历史
        self.track_histories = defaultdict(lambda: deque(maxlen=50))  # 保留50个历史点
        self.track_colors = {}
        self.track_speeds = defaultdict(list)
        
        # 性能统计
        self.frame_times = deque(maxlen=100)  # FPS计算
        self.detection_counts = deque(maxlen=100)
        self.tracking_accuracies = deque(maxlen=100)
        
        # 当前帧信息
        self.current_frame = 0
        self.current_scene = ""
        
        # 视频录制
        self.video_writer = None
        if save_video:
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.video_writer = cv2.VideoWriter(
                'visual/mctrack_realtime.mp4', fourcc, 20.0, self.canvas_size)
        
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, self.canvas_size[0], self.canvas_size[1])
    
    def get_track_color(self, track_id):
        """为每个轨迹分配唯一颜色"""
        if track_id not in self.track_colors:
            # 使用HSV色彩空间生成区分度高的颜色
            hue = (track_id * 137) % 360  # 黄金角度分布
            color_hsv = np.array([[[hue, 255, 255]]], dtype=np.uint8)
            color_bgr = cv2.cvtColor(color_hsv, cv2.COLOR_HSV2BGR)[0, 0]
            self.track_colors[track_id] = tuple(map(int, color_bgr))
        return self.track_colors[track_id]
    
    def world_to_pixel(self, world_pos):
        """将世界坐标转换为像素坐标"""
        x, y = world_pos[:2]
        px = int(self.origin[0] + x * self.scale)
        py = int(self.origin[1] - y * self.scale)  # Y轴翻转
        return (px, py)
    
    def draw_vehicle(self, canvas, center, lwh, yaw, track_id, velocity=None):
        """绘制车辆"""
        color = self.get_track_color(track_id)
        px, py = self.world_to_pixel(center)
        
        # 计算车辆四个角点
        l, w = lwh[0] * self.scale, lwh[1] * self.scale
        cos_yaw, sin_yaw = np.cos(yaw), np.sin(yaw)
        
        corners = np.array([
            [-l/2, -w/2], [l/2, -w/2], [l/2, w/2], [-l/2, w/2]
        ])
        
        # 旋转
        rotation_matrix = np.array([[cos_yaw, -sin_yaw], [sin_yaw, cos_yaw]])
        rotated_corners = corners @ rotation_matrix.T
        
        # 平移到车辆位置
        vehicle_corners = rotated_corners + np.array([px, py])
        vehicle_corners = vehicle_corners.astype(int)
        
        # 绘制车辆轮廓
        cv2.fillPoly(canvas, [vehicle_corners], color)
        cv2.polylines(canvas, [vehicle_corners], True, (255, 255, 255), 2)
        
        # 绘制朝向箭头
        arrow_length = max(l, w) * 0.6
        arrow_end = (
            int(px + arrow_length * cos_yaw),
            int(py + arrow_length * sin_yaw)
        )
        cv2.arrowedLine(canvas, (px, py), arrow_end, (255, 255, 255), 3, tipLength=0.3)
        
        # 绘制Track ID
        cv2.putText(canvas, f"ID:{track_id}", 
                   (px - 20, py - int(w/2) - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # 绘制速度信息
        if velocity is not None:
            speed = np.linalg.norm(velocity[:2]) * 3.6  # m/s to km/h
            cv2.putText(canvas, f"{speed:.1f}km/h", 
                       (px - 20, py + int(w/2) + 25),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
    
    def draw_trajectory(self, canvas, track_id):
        """绘制轨迹"""
        if track_id not in self.track_histories or len(self.track_histories[track_id]) < 2:
            return
        
        color = self.get_track_color(track_id)
        points = []
        
        for pos in self.track_histories[track_id]:
            points.append(self.world_to_pixel(pos))
        
        # 绘制轨迹线，颜色渐变表示时间
        for i in range(len(points) - 1):
            alpha = i / len(points)  # 透明度渐变
            thickness = max(1, int(3 * alpha))
            
            # 调整颜色亮度
            fade_color = tuple(int(c * (0.3 + 0.7 * alpha)) for c in color)
            cv2.line(canvas, points[i], points[i + 1], fade_color, thickness)
    
    def draw_performance_panel(self, canvas):
        """绘制性能面板"""
        panel_width = 350
        panel_height = 200
        panel_x = self.canvas_size[0] - panel_width - 10
        panel_y = 10
        
        # 绘制半透明背景
        overlay = canvas.copy()
        cv2.rectangle(overlay, (panel_x, panel_y), 
                     (panel_x + panel_width, panel_y + panel_height),
                     (0, 0, 0), -1)
        cv2.addWeighted(canvas, 0.7, overlay, 0.3, 0, canvas)
        
        # 绘制边框
        cv2.rectangle(canvas, (panel_x, panel_y), 
                     (panel_x + panel_width, panel_y + panel_height),
                     (255, 255, 255), 2)
        
        # 性能文本
        y_offset = panel_y + 30
        text_color = (255, 255, 255)
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.6
        
        # FPS
        fps = len(self.frame_times) / sum(self.frame_times) if self.frame_times else 0
        cv2.putText(canvas, f"FPS: {fps:.1f}", 
                   (panel_x + 10, y_offset), font, font_scale, text_color, 2)
        y_offset += 25
        
        # 检测数量
        avg_detections = np.mean(self.detection_counts) if self.detection_counts else 0
        cv2.putText(canvas, f"Avg Detections: {avg_detections:.1f}", 
                   (panel_x + 10, y_offset), font, font_scale, text_color, 2)
        y_offset += 25
        
        # 活跃轨迹数
        active_tracks = len(self.track_histories)
        cv2.putText(canvas, f"Active Tracks: {active_tracks}", 
                   (panel_x + 10, y_offset), font, font_scale, text_color, 2)
        y_offset += 25
        
        # 当前帧
        cv2.putText(canvas, f"Frame: {self.current_frame}", 
                   (panel_x + 10, y_offset), font, font_scale, text_color, 2)
        y_offset += 25
        
        # 场景名称
        cv2.putText(canvas, f"Scene: {self.current_scene[:15]}", 
                   (panel_x + 10, y_offset), font, font_scale, text_color, 2)
        
        # 绘制FPS曲线
        if len(self.frame_times) > 10:
            fps_history = []
            for i in range(len(self.frame_times)):
                window = list(self.frame_times)[max(0, i-9):i+1]
                fps_val = len(window) / sum(window) if sum(window) > 0 else 0
                fps_history.append(fps_val)
            
            # 绘制FPS曲线
            max_fps = max(fps_history) if fps_history else 30
            curve_y = panel_y + panel_height - 50
            curve_height = 30
            
            for i in range(len(fps_history) - 1):
                x1 = panel_x + 10 + i * 3
                x2 = panel_x + 10 + (i + 1) * 3
                y1 = int(curve_y - (fps_history[i] / max_fps) * curve_height)
                y2 = int(curve_y - (fps_history[i + 1] / max_fps) * curve_height)
                cv2.line(canvas, (x1, y1), (x2, y2), (0, 255, 0), 2)
    
    def draw_coordinate_system(self, canvas):
        """绘制坐标系"""
        # X轴 (红色)
        cv2.arrowedLine(canvas, self.origin, 
                       (self.origin[0] + 100, self.origin[1]),
                       (0, 0, 255), 3, tipLength=0.1)
        cv2.putText(canvas, "X", (self.origin[0] + 110, self.origin[1] + 5),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        # Y轴 (绿色)
        cv2.arrowedLine(canvas, self.origin, 
                       (self.origin[0], self.origin[1] - 100),
                       (0, 255, 0), 3, tipLength=0.1)
        cv2.putText(canvas, "Y", (self.origin[0] + 5, self.origin[1] - 110),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # 绘制网格
        grid_step = 50
        grid_color = (50, 50, 50)
        
        for i in range(-10, 11):
            if i == 0:
                continue
            # 垂直线
            x = self.origin[0] + i * grid_step
            cv2.line(canvas, (x, 0), (x, self.canvas_size[1]), grid_color, 1)
            
            # 水平线
            y = self.origin[1] + i * grid_step
            cv2.line(canvas, (0, y), (self.canvas_size[0], y), grid_color, 1)
    
    def update_frame(self, frame_data, scene_name, frame_number):
        """更新单帧可视化"""
        start_time = time.time()
        
        # 创建黑色画布
        canvas = np.zeros((self.canvas_size[1], self.canvas_size[0], 3), dtype=np.uint8)
        
        # 绘制坐标系和网格
        self.draw_coordinate_system(canvas)
        
        # 更新当前帧信息
        self.current_frame = frame_number
        self.current_scene = scene_name
        
        detection_count = 0
        
        # 处理检测结果
        if 'bboxes' in frame_data:
            for bbox_name, bbox_info in frame_data['bboxes'].items():
                detection_count += 1
                
                # 提取信息
                center = np.array(bbox_info.get('global_xyz', [0, 0, 0]))
                lwh = np.array(bbox_info.get('lwh', [4, 2, 1.5]))
                yaw = bbox_info.get('global_yaw', 0)
                velocity = np.array(bbox_info.get('global_velocity', [0, 0, 0]))
                score = bbox_info.get('detection_score', 0)
                category = bbox_info.get('category', 'unknown')
                
                # 生成track ID（实际使用中应该从跟踪器获取）
                track_id = hash(bbox_name) % 1000
                
                # 更新轨迹历史
                self.track_histories[track_id].append(center[:2])
                self.track_speeds[track_id].append(np.linalg.norm(velocity[:2]))
                
                # 绘制轨迹
                self.draw_trajectory(canvas, track_id)
                
                # 绘制车辆
                self.draw_vehicle(canvas, center, lwh, yaw, track_id, velocity)
        
        # 绘制性能面板
        self.draw_performance_panel(canvas)
        
        # 更新性能统计
        frame_time = time.time() - start_time
        self.frame_times.append(frame_time)
        self.detection_counts.append(detection_count)
        
        # 显示结果
        cv2.imshow(self.window_name, canvas)
        
        # 保存视频
        if self.video_writer:
            self.video_writer.write(canvas)
        
        return canvas

    def update_kitti_frame(self, frame_data, scene_name, frame_id):
        """更新KITTI格式的帧数据"""
        start_time = time.time()
        
        # 创建黑色画布
        canvas = np.zeros((self.canvas_size[1], self.canvas_size[0], 3), dtype=np.uint8)
        
        # 绘制坐标系和网格
        self.draw_coordinate_system(canvas)
        
        # 更新当前帧信息
        self.current_frame = frame_id
        self.current_scene = scene_name
        
        detection_count = len(frame_data)
        
        # 绘制车辆轨迹
        for track_data in frame_data:
            track_id = track_data['track_id']
            bbox_3d = track_data['bbox_3d']
            
            # KITTI坐标系转换：从camera坐标系转换为鸟瞰图坐标系
            # camera坐标系: x右, y下, z前
            # 鸟瞰图坐标系: x前, y左  
            x, y, z = bbox_3d['location']
            center = [z, -x]  # z->x (前), -x->y (左)
            
            l, w, h = bbox_3d['dimensions']
            lwh = [l, w, h]
            yaw = bbox_3d['rotation_y'] + np.pi/2  # 调整朝向角度
            
            # 更新轨迹历史
            self.track_histories[track_id].append(center)
            
            # 计算速度
            velocity = None
            if len(self.track_histories[track_id]) > 1:
                prev_pos = self.track_histories[track_id][-2]
                curr_pos = self.track_histories[track_id][-1]
                velocity = [curr_pos[0] - prev_pos[0], curr_pos[1] - prev_pos[1], 0]
                self.track_speeds[track_id].append(np.linalg.norm(velocity[:2]))
            
            # 绘制轨迹
            self.draw_trajectory(canvas, track_id)
            
            # 绘制车辆
            self.draw_vehicle(canvas, center, lwh, yaw, track_id, velocity)
        
        # 绘制性能面板
        self.draw_performance_panel(canvas)
        
        # 更新性能统计
        frame_time = time.time() - start_time
        self.frame_times.append(frame_time)
        self.detection_counts.append(detection_count)
        
        # 显示结果
        cv2.imshow(self.window_name, canvas)
        
        # 保存视频
        if self.video_writer:
            self.video_writer.write(canvas)
        
        return canvas
    
    def cleanup(self):
        """清理资源"""
        if self.video_writer:
            self.video_writer.release()
        cv2.destroyAllWindows()

def demo_visualization():
    """演示可视化工具"""
    print("MCTrack Real-Time Visualizer Demo")
    print("Press 'q' to quit, 'space' to pause")
    
    # 创建可视化器
    visualizer = RealTimeVisualizer(save_video=True)
    
    # 加载测试数据
    result_path = "results/kitti/val_car_summary.json"
    if not os.path.exists(result_path):
        print("No tracking results found. Please run MCTrack first:")
        print("python main.py --dataset kitti -e -p 1")
        return
    
    with open(result_path, 'r') as f:
        data = json.load(f)
    
    # 获取第一个场景
    scene_name = list(data.keys())[0]
    scene_data = data[scene_name]
    frames = sorted(scene_data.keys(), key=lambda x: int(x.split('_')[1]))
    
    print(f"Visualizing scene: {scene_name}")
    print(f"Total frames: {len(frames)}")
    
    paused = False
    frame_idx = 0
    
    try:
        while frame_idx < len(frames):
            if not paused:
                frame_name = frames[frame_idx]
                frame_data = scene_data[frame_name]
                
                # 更新可视化
                visualizer.update_frame(frame_data, scene_name, frame_idx)
                
                frame_idx += 1
                
                # 控制播放速度
                time.sleep(0.05)  # 20 FPS
            
            # 处理按键
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord(' '):  # 空格暂停/继续
                paused = not paused
                print("Paused" if paused else "Resumed")
            elif key == ord('r'):  # 重置
                frame_idx = 0
                visualizer.track_histories.clear()
                print("Reset visualization")
    
    except KeyboardInterrupt:
        print("Interrupted by user")
    
    finally:
        visualizer.cleanup()
        print("Visualization completed")

if __name__ == "__main__":
    demo_visualization()