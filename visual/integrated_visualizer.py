#!/usr/bin/env python3
"""
MCTrack 集成实时可视化模块
直接集成到跟踪流程中的高性能可视化工具
"""
import cv2
import numpy as np
import time
from collections import deque, defaultdict
import threading
import queue

class IntegratedVisualizer:
    def __init__(self, config, enable_gui=True, save_video=False):
        self.config = config
        self.enable_gui = enable_gui
        self.save_video = save_video
        
        if not self.enable_gui and not self.save_video:
            return
        
        # 可视化参数
        self.canvas_size = (1400, 1000)
        self.scale = 6
        self.origin = (700, 500)
        
        # 数据缓存
        self.track_histories = defaultdict(lambda: deque(maxlen=30))
        self.track_colors = {}
        self.track_velocities = defaultdict(list)
        
        # 性能监控
        self.fps_counter = deque(maxlen=50)
        self.processing_times = deque(maxlen=50)
        self.track_counts = deque(maxlen=100)
        
        # 异步渲染
        self.render_queue = queue.Queue(maxsize=10)
        self.render_thread = None
        self.running = False
        
        # 视频录制
        self.video_writer = None
        if save_video:
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            timestamp = int(time.time())
            video_path = f'visual/tracking_{timestamp}.mp4'
            self.video_writer = cv2.VideoWriter(
                video_path, fourcc, 15.0, self.canvas_size)
            print(f"Recording video to: {video_path}")
        
        if self.enable_gui:
            cv2.namedWindow("MCTrack Real-Time", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("MCTrack Real-Time", *self.canvas_size)
            
            # 启动渲染线程
            self.running = True
            self.render_thread = threading.Thread(target=self._render_loop, daemon=True)
            self.render_thread.start()
    
    def get_track_color(self, track_id):
        """获取轨迹颜色"""
        if track_id not in self.track_colors:
            np.random.seed(track_id)  # 确保颜色一致性
            color = tuple(np.random.randint(80, 255, 3).tolist())
            self.track_colors[track_id] = color
        return self.track_colors[track_id]
    
    def world_to_pixel(self, world_pos):
        """世界坐标到像素坐标"""
        x, y = world_pos[0], world_pos[1]
        px = int(self.origin[0] + x * self.scale)
        py = int(self.origin[1] - y * self.scale)
        return np.clip([px, py], [0, 0], 
                      [self.canvas_size[0]-1, self.canvas_size[1]-1])
    
    def visualize_frame(self, trajectories, detections, scene_info):
        """可视化当前帧"""
        if not (self.enable_gui or self.save_video):
            return
        
        frame_start = time.time()
        
        try:
            # 准备渲染数据
            render_data = {
                'trajectories': trajectories.copy() if trajectories else {},
                'detections': detections.copy() if detections else [],
                'scene_info': scene_info.copy(),
                'timestamp': time.time()
            }
            
            # 异步提交渲染任务
            if not self.render_queue.full():
                self.render_queue.put(render_data, block=False)
                
        except Exception as e:
            print(f"Visualization error: {e}")
        
        # 更新性能统计
        processing_time = time.time() - frame_start
        self.processing_times.append(processing_time)
    
    def _render_loop(self):
        """渲染循环（在单独线程中运行）"""
        while self.running:
            try:
                # 获取渲染数据
                data = self.render_queue.get(timeout=0.1)
                self._render_frame(data)
                
            except queue.Empty:
                continue
            except Exception as e:
                print(f"Render error: {e}")
    
    def _render_frame(self, data):
        """渲染单帧"""
        # 创建画布
        canvas = np.zeros((self.canvas_size[1], self.canvas_size[0], 3), dtype=np.uint8)
        
        # 绘制网格
        self._draw_grid(canvas)
        
        # 绘制轨迹和检测
        trajectories = data['trajectories']
        detections = data['detections']
        
        # 更新轨迹历史
        for traj_id, traj in trajectories.items():
            if hasattr(traj, 'bbox') and traj.bbox is not None:
                center = [traj.bbox.global_xyz[0], traj.bbox.global_xyz[1]]
                self.track_histories[traj_id].append(center)
                
                # 计算速度
                if len(self.track_histories[traj_id]) > 1:
                    prev_pos = self.track_histories[traj_id][-2]
                    curr_pos = self.track_histories[traj_id][-1]
                    velocity = np.linalg.norm(np.array(curr_pos) - np.array(prev_pos))
                    self.track_velocities[traj_id].append(velocity)
        
        # 绘制轨迹历史
        for traj_id, history in self.track_histories.items():
            if len(history) > 1:
                self._draw_trajectory(canvas, history, self.get_track_color(traj_id))
        
        # 绘制当前检测/轨迹
        for traj_id, traj in trajectories.items():
            if hasattr(traj, 'bbox') and traj.bbox is not None:
                self._draw_vehicle(canvas, traj.bbox, traj_id)
        
        # 绘制性能信息
        self._draw_performance_info(canvas, data['scene_info'])
        
        # 显示结果
        if self.enable_gui:
            cv2.imshow("MCTrack Real-Time", canvas)
            cv2.waitKey(1)
        
        # 保存视频
        if self.video_writer:
            self.video_writer.write(canvas)
        
        # 更新FPS
        self.fps_counter.append(time.time())
    
    def _draw_grid(self, canvas):
        """绘制网格和坐标系"""
        # 网格
        grid_step = 40
        grid_color = (30, 30, 30)
        
        # 垂直线
        for x in range(0, self.canvas_size[0], grid_step):
            cv2.line(canvas, (x, 0), (x, self.canvas_size[1]), grid_color, 1)
        
        # 水平线  
        for y in range(0, self.canvas_size[1], grid_step):
            cv2.line(canvas, (0, y), (self.canvas_size[0], y), grid_color, 1)
        
        # 坐标轴
        cv2.line(canvas, (self.origin[0], 0), (self.origin[0], self.canvas_size[1]), 
                (0, 100, 0), 2)  # Y轴
        cv2.line(canvas, (0, self.origin[1]), (self.canvas_size[0], self.origin[1]), 
                (0, 0, 100), 2)  # X轴
    
    def _draw_trajectory(self, canvas, history, color):
        """绘制轨迹"""
        points = []
        for pos in history:
            px, py = self.world_to_pixel(pos)
            points.append([px, py])
        
        points = np.array(points, dtype=np.int32)
        
        # 绘制轨迹线，带透明度渐变
        for i in range(len(points) - 1):
            alpha = (i + 1) / len(points)
            thickness = max(1, int(3 * alpha))
            fade_color = tuple(int(c * alpha) for c in color)
            cv2.line(canvas, tuple(points[i]), tuple(points[i+1]), fade_color, thickness)
    
    def _draw_vehicle(self, canvas, bbox, track_id):
        """绘制车辆"""
        color = self.get_track_color(track_id)
        
        # 获取车辆信息
        center = [bbox.global_xyz[0], bbox.global_xyz[1]]
        lwh = [bbox.lwh[0], bbox.lwh[1], bbox.lwh[2]]
        yaw = getattr(bbox, 'global_yaw', 0)
        
        px, py = self.world_to_pixel(center)
        
        # 计算车辆矩形
        l, w = lwh[0] * self.scale * 0.8, lwh[1] * self.scale * 0.8
        cos_yaw, sin_yaw = np.cos(yaw), np.sin(yaw)
        
        # 车辆四角
        corners = np.array([
            [-l/2, -w/2], [l/2, -w/2], [l/2, w/2], [-l/2, w/2]
        ])
        
        # 旋转
        rotation_matrix = np.array([[cos_yaw, -sin_yaw], [sin_yaw, cos_yaw]])
        rotated_corners = corners @ rotation_matrix.T + [px, py]
        rotated_corners = rotated_corners.astype(int)
        
        # 绘制车辆
        cv2.fillPoly(canvas, [rotated_corners], color)
        cv2.polylines(canvas, [rotated_corners], True, (255, 255, 255), 1)
        
        # 绘制朝向
        arrow_end = [px + int(l * 0.7 * cos_yaw), py + int(l * 0.7 * sin_yaw)]
        cv2.arrowedLine(canvas, (px, py), tuple(arrow_end), (255, 255, 255), 2)
        
        # Track ID
        cv2.putText(canvas, str(track_id), (px - 10, py - int(w/2) - 5),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
    
    def _draw_performance_info(self, canvas, scene_info):
        """绘制性能信息"""
        info_x, info_y = 10, 30
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.6
        color = (255, 255, 255)
        thickness = 1
        
        # FPS
        if len(self.fps_counter) > 10:
            fps = len(self.fps_counter) / (self.fps_counter[-1] - self.fps_counter[0])
            cv2.putText(canvas, f"FPS: {fps:.1f}", (info_x, info_y), 
                       font, font_scale, color, thickness)
        
        # 轨迹数量
        cv2.putText(canvas, f"Tracks: {len(self.track_histories)}", 
                   (info_x, info_y + 25), font, font_scale, color, thickness)
        
        # 场景信息
        if 'frame' in scene_info:
            cv2.putText(canvas, f"Frame: {scene_info['frame']}", 
                       (info_x, info_y + 50), font, font_scale, color, thickness)
        
        if 'scene' in scene_info:
            scene_name = str(scene_info['scene'])[:20]
            cv2.putText(canvas, f"Scene: {scene_name}", 
                       (info_x, info_y + 75), font, font_scale, color, thickness)
    
    def cleanup(self):
        """清理资源"""
        self.running = False
        
        if self.render_thread and self.render_thread.is_alive():
            self.render_thread.join(timeout=1.0)
        
        if self.video_writer:
            self.video_writer.release()
            print("Video saved successfully")
        
        if self.enable_gui:
            cv2.destroyAllWindows()

# 使用示例：在main.py中集成
def create_visualizer(config, enable_visualization=True):
    """创建可视化器实例"""
    if not enable_visualization:
        return None
    
    return IntegratedVisualizer(
        config=config,
        enable_gui=True,
        save_video=False  # 可以根据需要开启
    )

# 在跟踪循环中调用的接口
def visualize_tracking_frame(visualizer, trajectories, detections, scene_info):
    """在跟踪过程中调用的可视化接口"""
    if visualizer is not None:
        visualizer.visualize_frame(trajectories, detections, scene_info)