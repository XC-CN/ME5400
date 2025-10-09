#!/usr/bin/env python3
"""
KITTI PointPillars ROS节点
将MMDetection3D PointPillars检测结果转换为ROS MarkerArray消息并发布
"""

import rospy
import numpy as np
import json
import os
from pathlib import Path
from typing import Dict, List, Any, Optional
import tf.transformations as tf_trans

# ROS消息类型
from std_msgs.msg import Header, String
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import PointCloud2

# MMDetection3D相关
from mmdet3d.apis import LidarDet3DInferencer


class KittiPointPillarsROSNode:
    """KITTI PointPillars ROS节点"""
    
    def __init__(self):
        """初始化ROS节点"""
        rospy.init_node('kitti_pointpillars_detector', anonymous=True)
        
        # 获取ROS参数 - 使用相对路径
        import os
        current_dir = os.path.dirname(os.path.abspath(__file__))
        self.config_path = rospy.get_param('~config_path', 
            os.path.join(current_dir, 'configs', 'pointpillars', 'pointpillars_hv_secfpn_8xb6-160e_kitti-3d-3class.py'))
        self.checkpoint_path = rospy.get_param('~checkpoint_path',
            os.path.join(current_dir, 'checkpoints', 'pointpillars_hv_secfpn_6x8_160e_kitti-3d-3class.pth'))
        self.data_dir = rospy.get_param('~data_dir',
            os.path.join(current_dir, 'data', 'kitti', '2011_09_26_drive_0039_sync', '2011_09_26', '2011_09_26_drive_0039_sync', 'velodyne_points', 'data'))
        self.publish_rate = rospy.get_param('~publish_rate', 10.0)  # Hz
        self.confidence_threshold = rospy.get_param('~confidence_threshold', 0.05)
        self.frame_id = rospy.get_param('~frame_id', 'lidar')
        
        # 类别名称和颜色映射
        self.class_names = ['Car', 'Pedestrian', 'Cyclist']
        self.class_colors = {
            'Car': (1.0, 0.0, 0.0, 0.8),        # 红色
            'Pedestrian': (0.0, 1.0, 0.0, 0.8),  # 绿色
            'Cyclist': (0.0, 0.0, 1.0, 0.8)      # 蓝色
        }
        
        # 初始化推理器
        self._init_inferencer()
        
        # 创建ROS发布器
        self.marker_pub = rospy.Publisher('/detection/bboxes_3d', MarkerArray, queue_size=10)
        self.status_pub = rospy.Publisher('/detection/status', String, queue_size=10)
        self.kitti_tracking_pub = rospy.Publisher('/detection/kitti_tracking', String, queue_size=10)
        
        # 获取所有.bin文件
        self.bin_files = self._get_bin_files()
        self.current_frame_idx = 0
        
        rospy.loginfo(f"KITTI PointPillars ROS节点初始化完成")
        rospy.loginfo(f"配置文件: {self.config_path}")
        rospy.loginfo(f"检查点: {self.checkpoint_path}")
        rospy.loginfo(f"数据目录: {self.data_dir}")
        rospy.loginfo(f"发布频率: {self.publish_rate} Hz")
        rospy.loginfo(f"找到 {len(self.bin_files)} 个点云文件")
    
    def _init_inferencer(self):
        """初始化MMDetection3D推理器"""
        try:
            rospy.loginfo("正在初始化PointPillars推理器...")
            from mmengine.registry import init_default_scope
            init_default_scope('mmdet3d')
            
            self.inferencer = LidarDet3DInferencer(
                model=self.config_path,
                weights=self.checkpoint_path,
                device='cuda:0'
            )
            rospy.loginfo("推理器初始化成功")
        except Exception as e:
            rospy.logerr(f"推理器初始化失败: {e}")
            raise
    
    def _get_bin_files(self) -> List[str]:
        """获取所有.bin文件"""
        data_path = Path(self.data_dir)
        if not data_path.exists():
            rospy.logerr(f"数据目录不存在: {self.data_dir}")
            return []
        
        bin_files = sorted([f.name for f in data_path.glob('*.bin')])
        rospy.loginfo(f"找到 {len(bin_files)} 个.bin文件")
        return bin_files
    
    def _load_pointcloud(self, bin_file: str) -> np.ndarray:
        """加载点云数据"""
        bin_path = Path(self.data_dir) / bin_file
        if not bin_path.exists():
            rospy.logwarn(f"点云文件不存在: {bin_path}")
            return None
        
        try:
            points = np.fromfile(bin_path, dtype=np.float32).reshape(-1, 4)
            return points
        except Exception as e:
            rospy.logerr(f"加载点云失败: {e}")
            return None
    
    def _run_inference(self, bin_file: str) -> Dict[str, Any]:
        """运行推理"""
        try:
            # 运行推理 - 直接传递文件路径，就像之前工作的脚本一样
            bin_path = Path(self.data_dir) / bin_file
            result = self.inferencer(
                inputs=dict(points=str(bin_path.absolute())),
                show=True,
                wait_time=0,
                pred_score_thr=self.confidence_threshold,
                no_save_vis=True,
                no_save_pred=True
            )
            
            # 提取检测结果
            detections = self._extract_detections(result)
            return detections
            
        except Exception as e:
            rospy.logerr(f"推理失败: {e}")
            return {'bboxes_3d': [], 'scores_3d': [], 'labels_3d': [], 'class_names': []}
    
    def _extract_detections(self, result) -> Dict[str, Any]:
        """从推理结果中提取检测信息"""
        detections = {
            'bboxes_3d': [],
            'scores_3d': [],
            'labels_3d': [],
            'class_names': []
        }
        
        try:
            # 调试信息
            rospy.loginfo(f"推理结果类型: {type(result)}")
            if isinstance(result, dict):
                rospy.loginfo(f"结果字典键: {list(result.keys())}")
            
            # 处理字典格式的结果
            if isinstance(result, dict) and 'predictions' in result:
                predictions_list = result['predictions']
                rospy.loginfo(f"predictions类型: {type(predictions_list)}")
                rospy.loginfo(f"predictions长度: {len(predictions_list)}")
                
                # predictions是一个列表，取第一个元素
                if len(predictions_list) > 0:
                    result = predictions_list[0]
                    rospy.loginfo(f"第一个prediction类型: {type(result)}")
                    if isinstance(result, dict):
                        rospy.loginfo(f"prediction字典键: {list(result.keys())}")
                else:
                    rospy.logwarn("predictions列表为空")
                    return detections
            
            # 处理Det3DDataSample对象或字典
            pred_instances = None
            if hasattr(result, 'pred_instances_3d') and result.pred_instances_3d is not None:
                pred_instances = result.pred_instances_3d
            elif isinstance(result, dict) and 'pred_instances_3d' in result:
                pred_instances = result['pred_instances_3d']
            elif isinstance(result, dict) and 'bboxes_3d' in result:
                # 直接处理字典格式的结果
                pred_instances = result
            
            if pred_instances is not None:
                rospy.loginfo(f"pred_instances类型: {type(pred_instances)}")
                
                # 获取边界框
                bboxes = None
                if hasattr(pred_instances, 'bboxes_3d') and pred_instances.bboxes_3d is not None:
                    if hasattr(pred_instances.bboxes_3d, 'tensor'):
                        bboxes = pred_instances.bboxes_3d.tensor.cpu().numpy()
                    else:
                        bboxes = pred_instances.bboxes_3d.cpu().numpy()
                elif isinstance(pred_instances, dict) and 'bboxes_3d' in pred_instances:
                    bboxes = pred_instances['bboxes_3d']
                
                # 获取置信度分数
                scores = None
                if hasattr(pred_instances, 'scores_3d') and pred_instances.scores_3d is not None:
                    scores = pred_instances.scores_3d.cpu().numpy()
                elif isinstance(pred_instances, dict) and 'scores_3d' in pred_instances:
                    scores = pred_instances['scores_3d']
                
                # 获取标签
                labels = None
                if hasattr(pred_instances, 'labels_3d') and pred_instances.labels_3d is not None:
                    labels = pred_instances.labels_3d.cpu().numpy()
                elif isinstance(pred_instances, dict) and 'labels_3d' in pred_instances:
                    labels = pred_instances['labels_3d']
                
                rospy.loginfo(f"bboxes类型: {type(bboxes)}, 长度: {len(bboxes) if bboxes is not None else 'None'}")
                rospy.loginfo(f"scores类型: {type(scores)}, 长度: {len(scores) if scores is not None else 'None'}")
                rospy.loginfo(f"labels类型: {type(labels)}, 长度: {len(labels) if labels is not None else 'None'}")
                
                if scores is not None:
                    # 转换为numpy数组进行过滤
                    if isinstance(scores, list):
                        scores = np.array(scores)
                    if isinstance(bboxes, list):
                        bboxes = np.array(bboxes)
                    if isinstance(labels, list):
                        labels = np.array(labels)
                    
                    # 过滤低置信度检测
                    valid_indices = scores > self.confidence_threshold
                    rospy.loginfo(f"置信度阈值: {self.confidence_threshold}")
                    rospy.loginfo(f"有效检测数量: {np.sum(valid_indices)}")
                    
                    if np.any(valid_indices):
                        detections['scores_3d'] = scores[valid_indices].tolist()
                        if bboxes is not None:
                            detections['bboxes_3d'] = bboxes[valid_indices].tolist()
                        if labels is not None:
                            detections['labels_3d'] = labels[valid_indices].tolist()
                            detections['class_names'] = [self.class_names[label] for label in labels[valid_indices]]
                        
                        rospy.loginfo(f"提取到 {len(detections['bboxes_3d'])} 个检测结果")
                    else:
                        rospy.logwarn("没有检测结果超过置信度阈值")
                else:
                    rospy.logwarn("无法获取置信度分数")
            else:
                rospy.logwarn("无法获取pred_instances_3d")
        
        except Exception as e:
            rospy.logerr(f"提取检测结果失败: {e}")
            import traceback
            rospy.logerr(f"错误详情: {traceback.format_exc()}")
        
        return detections
    
    def _detections_to_markerarray(self, detections: Dict[str, Any], frame_id: str) -> MarkerArray:
        """将检测结果转换为MarkerArray消息"""
        marker_array = MarkerArray()
        
        # 创建时间戳
        current_time = rospy.Time.now()
        
        for i, (bbox, score, class_name) in enumerate(zip(
            detections['bboxes_3d'], 
            detections['scores_3d'], 
            detections['class_names']
        )):
            # 解析边界框参数 [x, y, z, l, w, h, yaw] (KITTI格式是7个参数)
            if len(bbox) == 7:
                x, y, z, length, width, height, yaw = bbox
                vx, vy = 0.0, 0.0  # 默认速度
            elif len(bbox) == 9:
                x, y, z, length, width, height, yaw, vx, vy = bbox
            else:
                rospy.logwarn(f"未知边界框格式，长度: {len(bbox)}")
                continue
            
            # 创建Marker
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = current_time
            marker.ns = "detection"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            # 设置位置 (边界框中心)
            marker.pose.position.x = float(x)
            marker.pose.position.y = float(y)
            marker.pose.position.z = float(z)
            
            # 设置姿态 (旋转)
            quat = tf_trans.quaternion_from_euler(0, 0, yaw)
            marker.pose.orientation.x = quat[0]
            marker.pose.orientation.y = quat[1]
            marker.pose.orientation.z = quat[2]
            marker.pose.orientation.w = quat[3]
            
            # 设置尺寸
            marker.scale.x = float(length)
            marker.scale.y = float(width)
            marker.scale.z = float(height)
            
            # 设置颜色
            color = self.class_colors.get(class_name, (1.0, 1.0, 1.0, 0.8))
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = color[3]
            
            # 设置生命周期 (0表示永久显示)
            marker.lifetime = rospy.Duration(0)
            marker.frame_locked = False
            
            marker_array.markers.append(marker)
        
        return marker_array
    
    def _detections_to_kitti_tracking(self, detections: Dict[str, Any], frame_id: int) -> str:
        """将检测结果转换为KITTI tracking格式"""
        kitti_lines = []
        
        for i, (bbox, score, class_name) in enumerate(zip(
            detections['bboxes_3d'], 
            detections['scores_3d'], 
            detections['class_names']
        )):
            # 解析边界框参数 [x, y, z, l, w, h, yaw] (KITTI格式是7个参数)
            if len(bbox) == 7:
                x, y, z, length, width, height, yaw = bbox
                vx, vy = 0.0, 0.0  # 默认速度
            elif len(bbox) == 9:
                x, y, z, length, width, height, yaw, vx, vy = bbox
            else:
                rospy.logwarn(f"未知边界框格式，长度: {len(bbox)}")
                continue
            
            # KITTI tracking格式: frame_id track_id type truncated occluded alpha bbox_left bbox_top bbox_right bbox_bottom height width length x y z rotation_y
            # 注意: 这里我们只有3D信息，2D边界框设为默认值
            track_id = i  # 简单的track_id分配
            truncated = 0.0  # 默认值
            occluded = 0  # 默认值
            alpha = yaw  # 观察角度
            bbox_left = 0.0  # 2D边界框需要相机投影，这里设为默认值
            bbox_top = 0.0
            bbox_right = 0.0
            bbox_bottom = 0.0
            
            kitti_line = f"{frame_id} {track_id} {class_name} {truncated} {occluded} {alpha:.6f} {bbox_left:.6f} {bbox_top:.6f} {bbox_right:.6f} {bbox_bottom:.6f} {height:.6f} {width:.6f} {length:.6f} {x:.6f} {y:.6f} {z:.6f} {yaw:.6f}"
            kitti_lines.append(kitti_line)
        
        return '\n'.join(kitti_lines)
    
    def _publish_status(self, status: str):
        """发布状态信息"""
        status_msg = String()
        status_msg.data = status
        self.status_pub.publish(status_msg)
    
    def run_continuous_inference(self):
        """运行连续推理"""
        rospy.loginfo("开始连续推理...")
        
        rate = rospy.Rate(self.publish_rate)
        
        while not rospy.is_shutdown() and self.current_frame_idx < len(self.bin_files):
            # 获取当前帧
            current_file = self.bin_files[self.current_frame_idx]
            
            # 发布状态
            self._publish_status(f"处理帧 {self.current_frame_idx + 1}/{len(self.bin_files)}: {current_file}")
            
            # 加载点云
            points = self._load_pointcloud(current_file)
            if points is None:
                self.current_frame_idx += 1
                continue
            
            # 运行推理
            detections = self._run_inference(current_file)
            
            # 转换为MarkerArray并发布
            marker_array = self._detections_to_markerarray(detections, self.frame_id)
            self.marker_pub.publish(marker_array)
            
            # 转换为KITTI tracking格式并发布
            kitti_tracking = self._detections_to_kitti_tracking(detections, self.current_frame_idx)
            kitti_msg = String()
            kitti_msg.data = kitti_tracking
            self.kitti_tracking_pub.publish(kitti_msg)
            
            # 发布检测统计信息
            num_detections = len(detections['bboxes_3d'])
            rospy.loginfo(f"帧 {self.current_frame_idx + 1}: 检测到 {num_detections} 个目标")
            
            # 移动到下一帧
            self.current_frame_idx += 1
            
            # 等待下一个发布周期
            rate.sleep()
        
        rospy.loginfo("连续推理完成")
        self._publish_status("推理完成")
    
    def run_single_frame(self, frame_idx: int = 0):
        """运行单帧推理"""
        if frame_idx >= len(self.bin_files):
            rospy.logerr(f"帧索引超出范围: {frame_idx}")
            return
        
        current_file = self.bin_files[frame_idx]
        rospy.loginfo(f"处理单帧: {current_file}")
        
        # 加载点云
        points = self._load_pointcloud(current_file)
        if points is None:
            return
        
        # 运行推理
        detections = self._run_inference(points)
        
        # 转换为MarkerArray并发布
        marker_array = self._detections_to_markerarray(detections, self.frame_id)
        self.marker_pub.publish(marker_array)
        
        # 发布检测统计信息
        num_detections = len(detections['bboxes_3d'])
        rospy.loginfo(f"检测到 {num_detections} 个目标")
        
        # 保持节点运行
        rospy.spin()


def main():
    """主函数"""
    try:
        # 创建ROS节点
        node = KittiPointPillarsROSNode()
        
        # 获取运行模式
        mode = rospy.get_param('~mode', 'continuous')  # 'continuous' 或 'single'
        
        if mode == 'continuous':
            node.run_continuous_inference()
        else:
            frame_idx = rospy.get_param('~frame_idx', 0)
            node.run_single_frame(frame_idx)
            
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS节点被中断")
    except Exception as e:
        rospy.logerr(f"节点运行错误: {e}")


if __name__ == '__main__':
    main()
