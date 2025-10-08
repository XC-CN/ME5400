#!/usr/bin/env python3
"""
KITTI数据集PointPillars推理脚本
支持单帧推理和批量处理
"""

import os
import sys
import json
import numpy as np
import open3d as o3d
from pathlib import Path
import argparse
from typing import List, Dict, Any

# 添加MMDetection3D路径
sys.path.append('/home/zzy/mmdet3d_centerpoint')

from mmdet3d.apis import LidarDet3DInferencer
from mmdet3d.structures import Det3DDataSample
from mmdet3d.visualization import Det3DLocalVisualizer


class KITTIPointPillarsInference:
    """KITTI数据集PointPillars推理类"""
    
    def __init__(self, config_path: str, checkpoint_path: str):
        """
        初始化推理器
        
        Args:
            config_path: 配置文件路径
            checkpoint_path: 模型权重路径
        """
        self.config_path = config_path
        self.checkpoint_path = checkpoint_path
        
        # 初始化推理器
        print("正在初始化PointPillars推理器...")
        self.inferencer = LidarDet3DInferencer(
            model=config_path,
            weights=checkpoint_path,
            device='cuda:0'
        )
        
        # KITTI类别定义
        self.class_names = ['Pedestrian', 'Cyclist', 'Car']
        self.class_colors = {
            'Pedestrian': [1, 0, 0],    # 红色
            'Cyclist': [0, 1, 0],       # 绿色  
            'Car': [0, 0, 1]            # 蓝色
        }
        
        print("PointPillars推理器初始化完成！")
    
    def load_kitti_pointcloud(self, bin_path: str) -> np.ndarray:
        """
        加载KITTI点云数据
        
        Args:
            bin_path: .bin文件路径
            
        Returns:
            point_cloud: 点云数据 (N, 4) [x, y, z, intensity]
        """
        # KITTI点云格式: 每个点4个float32值 (x, y, z, intensity)
        points = np.fromfile(bin_path, dtype=np.float32).reshape(-1, 4)
        return points
    
    def single_frame_inference(self, bin_path: str, save_result: bool = True) -> Dict[str, Any]:
        """
        单帧推理
        
        Args:
            bin_path: 点云文件路径
            save_result: 是否保存结果
            
        Returns:
            result: 检测结果
        """
        print(f"正在处理: {bin_path}")
        
        # 加载点云
        points = self.load_kitti_pointcloud(bin_path)
        print(f"点云数量: {len(points)}")
        
        # 创建正确的数据格式
        data_dict = {
            'points': points,
            'pts_filename': bin_path,
            'img_metas': {
                'lidar2img': None,
                'sample_idx': 0,
                'pts_filename': bin_path
            }
        }
        
        # 执行推理
        result = self.inferencer(data_dict, return_datasamples=True)
        
        # 直接查看推理器的原始输出
        print(f"推理结果类型: {type(result)}")
        if isinstance(result, dict) and 'predictions' in result:
            pred = result['predictions']
            if hasattr(pred, 'pred_instances_3d') and pred.pred_instances_3d is not None:
                instances = pred.pred_instances_3d
                print(f"原始检测数量: {len(instances.bboxes_3d) if hasattr(instances, 'bboxes_3d') and instances.bboxes_3d is not None else 0}")
                if hasattr(instances, 'scores_3d') and instances.scores_3d is not None:
                    scores = instances.scores_3d.cpu().numpy()
                    print(f"置信度范围: [{scores.min():.3f}, {scores.max():.3f}]")
                    print(f"置信度>0.1的数量: {np.sum(scores > 0.1)}")
                    print(f"置信度>0.05的数量: {np.sum(scores > 0.05)}")
                    print(f"置信度>0.01的数量: {np.sum(scores > 0.01)}")
        
        # 提取检测结果
        detections = self._extract_detections_from_dict(result)
        
        if save_result:
            # 保存结果
            result_path = bin_path.replace('.bin', '_pointpillars_result.json')
            with open(result_path, 'w') as f:
                json.dump(detections, f, indent=2)
            print(f"结果已保存到: {result_path}")
        
        return detections
    
    def _extract_detections_from_dict(self, result: Dict) -> Dict[str, Any]:
        """
        从字典格式的结果中提取检测结果
        
        Args:
            result: 推理结果字典
            
        Returns:
            detections: 检测结果字典
        """
        detections = {
            'bboxes_3d': [],
            'scores_3d': [],
            'labels_3d': [],
            'class_names': []
        }
        
        print(f"结果字典键: {result.keys()}")
        
        # 检查是否有预测结果
        if 'predictions' in result and len(result['predictions']) > 0:
            pred = result['predictions'][0]  # 取第一个预测结果
            print(f"预测结果类型: {type(pred)}")
            
            # 使用Det3DDataSample对象的方法
            if hasattr(pred, 'pred_instances_3d') and pred.pred_instances_3d is not None:
                pred_instances = pred.pred_instances_3d
                print(f"预测实例属性: {dir(pred_instances)}")
                
                if hasattr(pred_instances, 'bboxes_3d') and pred_instances.bboxes_3d is not None:
                    bboxes = pred_instances.bboxes_3d.tensor.cpu().numpy()
                    print(f"检测到 {len(bboxes)} 个边界框")
                    detections['bboxes_3d'] = bboxes.tolist()
                
                if hasattr(pred_instances, 'scores_3d') and pred_instances.scores_3d is not None:
                    scores = pred_instances.scores_3d.cpu().numpy()
                    print(f"置信度分数: {scores}")
                    # 降低置信度阈值
                    valid_indices = scores > 0.1
                    if np.any(valid_indices):
                        detections['scores_3d'] = scores[valid_indices].tolist()
                        if hasattr(pred_instances, 'bboxes_3d') and pred_instances.bboxes_3d is not None:
                            bboxes = pred_instances.bboxes_3d.tensor.cpu().numpy()
                            detections['bboxes_3d'] = bboxes[valid_indices].tolist()
                        if hasattr(pred_instances, 'labels_3d') and pred_instances.labels_3d is not None:
                            labels = pred_instances.labels_3d.cpu().numpy()
                            detections['labels_3d'] = labels[valid_indices].tolist()
                            detections['class_names'] = [self.class_names[label] for label in labels[valid_indices]]
                    else:
                        print("没有检测到高置信度的目标")
                else:
                    print("没有置信度分数")
                
                if hasattr(pred_instances, 'labels_3d') and pred_instances.labels_3d is not None and len(detections['labels_3d']) == 0:
                    labels = pred_instances.labels_3d.cpu().numpy()
                    detections['labels_3d'] = labels.tolist()
                    detections['class_names'] = [self.class_names[label] for label in labels]
        else:
            print("没有预测结果")
        
        return detections
    
    def _extract_detections(self, result: Det3DDataSample) -> Dict[str, Any]:
        """
        提取检测结果
        
        Args:
            result: 推理结果
            
        Returns:
            detections: 检测结果字典
        """
        detections = {
            'bboxes_3d': [],
            'scores_3d': [],
            'labels_3d': [],
            'class_names': []
        }
        
        print(f"结果类型: {type(result)}")
        print(f"结果属性: {dir(result)}")
        
        # 处理字典格式的结果
        if isinstance(result, dict) and 'predictions' in result:
            result = result['predictions']
            print(f"提取预测结果类型: {type(result)}")
        
        if hasattr(result, 'pred_instances_3d') and result.pred_instances_3d is not None:
            pred_instances = result.pred_instances_3d
            print(f"预测实例类型: {type(pred_instances)}")
            print(f"预测实例属性: {dir(pred_instances)}")
            
            # 先获取所有原始检测结果
            if hasattr(pred_instances, 'bboxes_3d') and pred_instances.bboxes_3d is not None:
                bboxes = pred_instances.bboxes_3d.tensor.cpu().numpy()
                print(f"原始检测数量: {len(bboxes)} 个边界框")
            else:
                bboxes = None
                print("没有检测到边界框")
            
            if hasattr(pred_instances, 'scores_3d') and pred_instances.scores_3d is not None:
                scores = pred_instances.scores_3d.cpu().numpy()
                print(f"置信度分数: {scores}")
                print(f"置信度范围: [{scores.min():.3f}, {scores.max():.3f}]")
                
                # 进一步降低置信度阈值
                valid_indices = scores > 0.01  # 降低阈值到0.01，显示更多检测结果
                print(f"过滤后检测数量: {np.sum(valid_indices)}")
                
                if np.any(valid_indices):
                    detections['scores_3d'] = scores[valid_indices].tolist()
                    if bboxes is not None:
                        detections['bboxes_3d'] = bboxes[valid_indices].tolist()
                    if hasattr(pred_instances, 'labels_3d') and pred_instances.labels_3d is not None:
                        labels = pred_instances.labels_3d.cpu().numpy()
                        detections['labels_3d'] = labels[valid_indices].tolist()
                        detections['class_names'] = [self.class_names[label] for label in labels[valid_indices]]
                else:
                    print("没有检测到高置信度的目标")
            else:
                print("没有置信度分数")
            
            if hasattr(pred_instances, 'labels_3d') and pred_instances.labels_3d is not None and len(detections['labels_3d']) == 0:
                labels = pred_instances.labels_3d.cpu().numpy()
                detections['labels_3d'] = labels.tolist()
                detections['class_names'] = [self.class_names[label] for label in labels]
        else:
            print("没有预测实例")
        
        return detections
    
    def batch_inference(self, data_dir: str, output_dir: str = None) -> Dict[str, Any]:
        """
        批量推理
        
        Args:
            data_dir: 数据目录
            output_dir: 输出目录
            
        Returns:
            summary: 处理摘要
        """
        if output_dir is None:
            output_dir = os.path.join(data_dir, 'pointpillars_results')
        
        os.makedirs(output_dir, exist_ok=True)
        
        # 获取所有.bin文件
        bin_files = sorted([f for f in os.listdir(data_dir) if f.endswith('.bin')])
        
        print(f"找到 {len(bin_files)} 个点云文件")
        
        results = []
        total_detections = 0
        
        for i, bin_file in enumerate(bin_files):
            bin_path = os.path.join(data_dir, bin_file)
            
            try:
                # 单帧推理
                result = self.single_frame_inference(bin_path, save_result=False)
                
                # 保存结果
                result_file = os.path.join(output_dir, bin_file.replace('.bin', '_result.json'))
                with open(result_file, 'w') as f:
                    json.dump(result, f, indent=2)
                
                # 统计信息
                num_detections = len(result['bboxes_3d'])
                total_detections += num_detections
                
                results.append({
                    'file': bin_file,
                    'num_detections': num_detections,
                    'detections': result
                })
                
                print(f"进度: {i+1}/{len(bin_files)} - {bin_file} - 检测到 {num_detections} 个目标")
                
            except Exception as e:
                print(f"处理 {bin_file} 时出错: {e}")
                continue
        
        # 保存处理摘要
        summary = {
            'total_frames': len(bin_files),
            'processed_frames': len(results),
            'total_detections': total_detections,
            'average_detections_per_frame': total_detections / len(results) if results else 0,
            'results': results
        }
        
        summary_file = os.path.join(output_dir, 'processing_summary.json')
        with open(summary_file, 'w') as f:
            json.dump(summary, f, indent=2)
        
        print(f"\n批量处理完成！")
        print(f"处理帧数: {summary['processed_frames']}/{summary['total_frames']}")
        print(f"总检测数: {summary['total_detections']}")
        print(f"平均每帧检测数: {summary['average_detections_per_frame']:.2f}")
        print(f"结果保存在: {output_dir}")
        
        return summary
    
    def visualize_detection(self, bin_path: str, result: Dict[str, Any], 
                          show_bbox: bool = True, show_points: bool = True):
        """
        可视化检测结果
        
        Args:
            bin_path: 点云文件路径
            result: 检测结果
            show_bbox: 是否显示边界框
            show_points: 是否显示点云
        """
        # 加载点云
        points = self.load_kitti_pointcloud(bin_path)
        
        # 创建Open3D点云
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points[:, :3])
        
        # 设置点云颜色为白色
        pcd.paint_uniform_color([1.0, 1.0, 1.0])  # 白色
        
        # 创建可视化器
        vis = o3d.visualization.Visualizer()
        vis.create_window(window_name="KITTI PointPillars Detection", width=1200, height=800)
        
        # 设置背景为黑色
        vis.get_render_option().background_color = np.array([0.0, 0.0, 0.0])  # 黑色背景
        
        # 设置点云渲染选项
        render_option = vis.get_render_option()
        render_option.point_size = 1.0  # 减小点云大小
        render_option.show_coordinate_frame = False
        
        if show_points:
            vis.add_geometry(pcd)
        
        # 添加检测框
        if show_bbox and result['bboxes_3d']:
            print(f"正在绘制 {len(result['bboxes_3d'])} 个边界框")
            for i, (bbox, score, label) in enumerate(zip(
                result['bboxes_3d'], 
                result['scores_3d'], 
                result['labels_3d']
            )):
                if score > 0.1:  # 降低置信度阈值
                    print(f"绘制边界框 {i}: 类别={self.class_names[label]}, 置信度={score:.3f}")
                    # 创建边界框
                    bbox_3d = self._create_bbox_3d(bbox, score, label)
                    if bbox_3d is not None:
                        vis.add_geometry(bbox_3d)
        
        # 设置BEV视角 - 从上往下看的俯视图
        ctr = vis.get_view_control()
        ctr.set_front([0, 0, 1])   # 朝向正z轴（向下看）
        ctr.set_up([0, 1, 0])      # 上方向为正y轴（保持正确的方向）
        ctr.set_lookat([0, 0, 0])  # 看向原点
        ctr.set_zoom(0.8)          # 调整缩放
        
        # 运行可视化
        vis.run()
        vis.destroy_window()
    
    def _create_bbox_3d(self, bbox: List[float], score: float, label: int) -> o3d.geometry.LineSet:
        """
        创建3D边界框
        
        Args:
            bbox: 边界框参数 [x, y, z, w, l, h, yaw] (注意：这里w和l的顺序需要修正)
            score: 置信度
            label: 类别标签
            
        Returns:
            bbox_3d: 3D边界框
        """
        try:
            x, y, z, w, l, h, yaw = bbox
            
            # 修正：交换宽度和长度的位置，因为PointPillars输出的格式是 [x, y, z, w, l, h, yaw]
            # 但KITTI标准格式是 [x, y, z, l, w, h, yaw]，所以需要交换w和l
            actual_length = w  # PointPillars的w实际上是长度
            actual_width = l   # PointPillars的l实际上是宽度
            
            print(f"原始参数: x={x:.2f}, y={y:.2f}, z={z:.2f}, w={w:.2f}, l={l:.2f}, h={h:.2f}, yaw={yaw:.2f}")
            print(f"修正后: 长度={actual_length:.2f}m, 宽度={actual_width:.2f}m, 高度={h:.2f}m")
            
            # 创建8个顶点 (KITTI坐标系: x前, y左, z上)
            # 修正：边界框的底部应该在地面上，顶部在z+h处
            corners = np.array([
                [-actual_length/2, -actual_width/2, 0],     # 0: 后左下 (地面)
                [actual_length/2, -actual_width/2, 0],      # 1: 前左下 (地面)
                [actual_length/2, actual_width/2, 0],       # 2: 前右下 (地面)
                [-actual_length/2, actual_width/2, 0],      # 3: 后右下 (地面)
                [-actual_length/2, -actual_width/2, h],     # 4: 后左上 (车顶)
                [actual_length/2, -actual_width/2, h],      # 5: 前左上 (车顶)
                [actual_length/2, actual_width/2, h],       # 6: 前右上 (车顶)
                [-actual_length/2, actual_width/2, h]       # 7: 后右上 (车顶)
            ])
            
            # 旋转矩阵 (绕z轴旋转)
            cos_yaw = np.cos(yaw)
            sin_yaw = np.sin(yaw)
            rotation_matrix = np.array([
                [cos_yaw, -sin_yaw, 0],
                [sin_yaw, cos_yaw, 0],
                [0, 0, 1]
            ])
            
            # 应用旋转
            corners = corners @ rotation_matrix.T
            
            # 应用平移
            corners += np.array([x, y, z])
            
            # 创建线框连接
            lines = [
                [0, 1], [1, 2], [2, 3], [3, 0],  # 底面
                [4, 5], [5, 6], [6, 7], [7, 4],  # 顶面
                [0, 4], [1, 5], [2, 6], [3, 7]   # 连接线
            ]
            
            line_set = o3d.geometry.LineSet()
            line_set.points = o3d.utility.Vector3dVector(corners)
            line_set.lines = o3d.utility.Vector2iVector(lines)
            
            # 根据类别设置橙红黄暖色调
            class_name = self.class_names[label]
            if class_name == 'Car':
                base_color = [1.0, 0.5, 0.0]  # 橙色
            elif class_name == 'Pedestrian':
                base_color = [1.0, 0.2, 0.0]  # 红色
            elif class_name == 'Cyclist':
                base_color = [1.0, 0.8, 0.0]  # 黄色
            else:
                base_color = [0.9, 0.9, 0.9]  # 浅灰色
            
            # 根据置信度调整颜色亮度，但保持柔和
            brightness = 0.6 + 0.4 * score  # 基础亮度0.6，最高1.0
            color = [c * brightness for c in base_color]
            
            line_set.colors = o3d.utility.Vector3dVector([color] * len(lines))
            
            return line_set
            
        except Exception as e:
            print(f"创建边界框时出错: {e}")
            print(f"bbox参数: {bbox}")
            return None


def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='KITTI PointPillars推理')
    parser.add_argument('--config', type=str, 
                       default='configs/pointpillars/pointpillars_hv_secfpn_8xb6-160e_kitti-3d-3class.py',
                       help='配置文件路径')
    parser.add_argument('--checkpoint', type=str,
                       default='checkpoints/pointpillars_hv_secfpn_6x8_160e_kitti-3d-3class.pth',
                       help='模型权重路径')
    parser.add_argument('--data_dir', type=str, 
                       default='data/kitti/2011_09_26_drive_0039_sync/2011_09_26/2011_09_26_drive_0039_sync/velodyne_points/data',
                       help='KITTI数据目录')
    parser.add_argument('--output_dir', type=str, default='kitti_pointpillars_results',
                       help='输出目录')
    parser.add_argument('--mode', type=str, choices=['single', 'batch', 'visualize'], 
                       default='batch', help='运行模式')
    parser.add_argument('--frame_idx', type=int, default=0, help='单帧模式下的帧索引')
    
    args = parser.parse_args()
    
    # 检查文件是否存在
    if not os.path.exists(args.config):
        print(f"配置文件不存在: {args.config}")
        return
    
    if not os.path.exists(args.checkpoint):
        print(f"模型权重不存在: {args.checkpoint}")
        print("请先下载预训练模型:")
        print("wget https://download.openmmlab.com/mmdetection3d/v1.0.0_models/pointpillars/hv_pointpillars_secfpn_6x8_160e_kitti-3d-3class/hv_pointpillars_secfpn_6x8_160e_kitti-3d-3class_20220301_150306-37dc2420.pth -O checkpoints/pointpillars_hv_secfpn_6x8_160e_kitti-3d-3class.pth")
        return
    
    if not os.path.exists(args.data_dir):
        print(f"数据目录不存在: {args.data_dir}")
        return
    
    # 初始化推理器
    try:
        inference = KITTIPointPillarsInference(args.config, args.checkpoint)
    except Exception as e:
        print(f"初始化推理器失败: {e}")
        return
    
    # 根据模式运行
    if args.mode == 'single':
        # 单帧推理
        bin_files = sorted([f for f in os.listdir(args.data_dir) if f.endswith('.bin')])
        if args.frame_idx < len(bin_files):
            bin_path = os.path.join(args.data_dir, bin_files[args.frame_idx])
            result = inference.single_frame_inference(bin_path)
            print(f"检测结果: {result}")
        else:
            print(f"帧索引超出范围: {args.frame_idx} >= {len(bin_files)}")
    
    elif args.mode == 'batch':
        # 批量推理
        inference.batch_inference(args.data_dir, args.output_dir)
    
    elif args.mode == 'visualize':
        # 可视化
        bin_files = sorted([f for f in os.listdir(args.data_dir) if f.endswith('.bin')])
        if args.frame_idx < len(bin_files):
            bin_path = os.path.join(args.data_dir, bin_files[args.frame_idx])
            result = inference.single_frame_inference(bin_path, save_result=False)
            inference.visualize_detection(bin_path, result)
        else:
            print(f"帧索引超出范围: {args.frame_idx} >= {len(bin_files)}")


if __name__ == '__main__':
    main()
