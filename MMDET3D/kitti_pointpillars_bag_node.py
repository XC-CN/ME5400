#!/usr/bin/env python3
"""
KITTI PointPillars ROS节点 - Bag文件版本
订阅ROS bag文件中的点云数据，运行MMDetection3D PointPillars检测
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
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2

# MMDetection3D相关
from mmdet3d.apis import LidarDet3DInferencer


class KittiPointPillarsBagNode:
    """KITTI PointPillars ROS节点 - Bag文件版本"""
    
    def __init__(self):
        """初始化ROS节点"""
        rospy.init_node('kitti_pointpillars_bag_detector', anonymous=True)
        
        # 获取ROS参数
        import os
        current_dir = os.path.dirname(os.path.abspath(__file__))
        self.config_path = rospy.get_param('~config_path', 
            os.path.join(current_dir, 'configs', 'pointpillars', 'pointpillars_hv_secfpn_8xb6-160e_kitti-3d-3class.py'))
        self.checkpoint_path = rospy.get_param('~checkpoint_path',
            os.path.join(current_dir, 'checkpoints', 'pointpillars_hv_secfpn_6x8_160e_kitti-3d-3class.pth'))
        self.bag_path = rospy.get_param('~bag_path',
            os.path.join(current_dir, 'data', 'kitti', 'seq_0019_with_det.bag'))
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
        self.pointcloud_pub = rospy.Publisher('/detection/pointcloud', PointCloud2, queue_size=10)
        
        # 订阅点云话题
        self.pointcloud_sub = rospy.Subscriber('/kitti/velo/pointcloud', PointCloud2, self._pointcloud_callback)
        
        # 统计信息
        self.frame_count = 0
        self.last_time = rospy.Time.now()
        
        rospy.loginfo(f"KITTI PointPillars Bag ROS节点初始化完成")
        rospy.loginfo(f"配置文件: {self.config_path}")
        rospy.loginfo(f"检查点: {self.checkpoint_path}")
        rospy.loginfo(f"Bag文件: {self.bag_path}")
        rospy.loginfo(f"订阅话题: /kitti/velo/pointcloud")
        rospy.loginfo(f"发布频率: {self.publish_rate} Hz")
    
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
    
    def _pointcloud_callback(self, msg: PointCloud2):
        """点云数据回调函数"""
        try:
            self.frame_count += 1
            
            # 发布状态
            self._publish_status(f"处理帧 {self.frame_count}: {msg.header.stamp}")
            
            # 转换点云数据
            points = self._pointcloud2_to_numpy(msg)
            if points is None:
                rospy.logwarn("点云转换失败")
                return
            
            # 运行推理
            detections = self._run_inference(points, msg.header.stamp)
            
            # 转换为MarkerArray并发布
            marker_array = self._detections_to_markerarray(detections, self.frame_id)
            self.marker_pub.publish(marker_array)
            
            # 转换为KITTI tracking格式并发布
            kitti_tracking = self._detections_to_kitti_tracking(detections, self.frame_count)
            kitti_msg = String()
            kitti_msg.data = kitti_tracking
            self.kitti_tracking_pub.publish(kitti_msg)
            
            # 发布原始点云（可选）
            self.pointcloud_pub.publish(msg)
            
            # 发布检测统计信息
            num_detections = len(detections['bboxes_3d'])
            rospy.loginfo(f"帧 {self.frame_count}: 检测到 {num_detections} 个目标")
            
            # 频率控制
            current_time = rospy.Time.now()
            dt = (current_time - self.last_time).to_sec()
            if dt < 1.0 / self.publish_rate:
                rospy.sleep(1.0 / self.publish_rate - dt)
            self.last_time = rospy.Time.now()
            
        except Exception as e:
            rospy.logerr(f"点云处理失败: {e}")
            import traceback
            rospy.logerr(f"错误详情: {traceback.format_exc()}")
    
    def _pointcloud2_to_numpy(self, msg: PointCloud2) -> Optional[np.ndarray]:
        """将PointCloud2消息转换为numpy数组"""
        try:
            # 解析点云数据
            points_list = []
            for data in point_cloud2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True):
                points_list.append([data[0], data[1], data[2], data[3]])  # [x, y, z, intensity]
            
            if len(points_list) == 0:
                rospy.logwarn("点云为空")
                return None
            
            points = np.array(points_list, dtype=np.float32)
            rospy.loginfo(f"转换得到 {len(points)} 个点")
            return points
            
        except Exception as e:
            rospy.logerr(f"点云转换失败: {e}")
            return None
    
    def _run_inference(self, points: np.ndarray, timestamp) -> Dict[str, Any]:
        """运行推理"""
        try:
            # 保存临时点云文件
            temp_file = f"/tmp/kitti_points_{timestamp.secs}_{timestamp.nsecs}.bin"
            points.astype(np.float32).tofile(temp_file)
            
            # 运行推理
            result = self.inferencer(
                inputs=dict(points=temp_file),
                show=True,
                wait_time=0,
                pred_score_thr=self.confidence_threshold,
                no_save_vis=True,
                no_save_pred=True
            )
            
            # 清理临时文件
            os.remove(temp_file)
            
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
            # 处理字典格式的结果
            if isinstance(result, dict) and 'predictions' in result:
                predictions_list = result['predictions']
                if len(predictions_list) > 0:
                    result = predictions_list[0]
            
            # 处理Det3DDataSample对象或字典
            pred_instances = None
            if hasattr(result, 'pred_instances_3d') and result.pred_instances_3d is not None:
                pred_instances = result.pred_instances_3d
            elif isinstance(result, dict) and 'pred_instances_3d' in result:
                pred_instances = result['pred_instances_3d']
            elif isinstance(result, dict) and 'bboxes_3d' in result:
                pred_instances = result
            
            if pred_instances is not None:
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
                    
                    if np.any(valid_indices):
                        detections['scores_3d'] = scores[valid_indices].tolist()
                        if bboxes is not None:
                            detections['bboxes_3d'] = bboxes[valid_indices].tolist()
                        if labels is not None:
                            detections['labels_3d'] = labels[valid_indices].tolist()
                            detections['class_names'] = [self.class_names[label] for label in labels[valid_indices]]
        
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
            # 解析边界框参数 [x, y, z, l, w, h, yaw]
            if len(bbox) == 7:
                x, y, z, length, width, height, yaw = bbox
            elif len(bbox) == 9:
                x, y, z, length, width, height, yaw, vx, vy = bbox
                vx, vy = 0.0, 0.0
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
            
            # 设置生命周期
            marker.lifetime = rospy.Duration(0.1)  # 100ms
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
            # 解析边界框参数
            if len(bbox) == 7:
                x, y, z, length, width, height, yaw = bbox
            elif len(bbox) == 9:
                x, y, z, length, width, height, yaw, vx, vy = bbox
                vx, vy = 0.0, 0.0
            else:
                continue
            
            # KITTI tracking格式
            track_id = i
            truncated = 0.0
            occluded = 0
            alpha = yaw
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


def main():
    """主函数"""
    try:
        # 创建ROS节点
        node = KittiPointPillarsBagNode()
        
        rospy.loginfo("等待点云数据...")
        rospy.spin()
            
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS节点被中断")
    except Exception as e:
        rospy.logerr(f"节点运行错误: {e}")


if __name__ == '__main__':
    main()
