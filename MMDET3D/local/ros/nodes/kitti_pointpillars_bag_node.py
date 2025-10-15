#!/usr/bin/env python3
"""
KITTI PointPillars ROS节点 - Bag文件版本
订阅ROS bag文件中的点云数据，运行MMDetection3D PointPillars检测
"""

import os
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import rospy
import tf.transformations as tf_trans
import torch
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point
from std_msgs.msg import Header, String
from visualization_msgs.msg import Marker, MarkerArray

# 追加 catkin_ws Python 包路径，便于直接运行脚本时找到自定义消息
CATKIN_PYTHON_PATH = Path(__file__).resolve().parents[4] / 'catkin_ws' / 'devel' / 'lib' / 'python3/dist-packages'
if CATKIN_PYTHON_PATH.exists():
    sys.path.insert(0, str(CATKIN_PYTHON_PATH))

# MMDetection3D相关
from mmdet3d.apis import LidarDet3DInferencer
from mmdet3d.structures import Box3DMode, LiDARInstance3DBoxes

# 自定义消息
try:
    from kitti_tracklets_viz.msg import Detection3D, Detection3DArray
except ImportError as exc:
    raise ImportError(
        "无法导入 kitti_tracklets_viz 消息。请先编译 catkin_ws（运行 ./Scripts/build_catkin_ws.sh），"
        "并确认已生成 devel/lib/python3/dist-packages。"
    ) from exc


class KittiPointPillarsBagNode:
    """KITTI PointPillars ROS节点 - Bag文件版本"""
    
    def __init__(self):
        """初始化ROS节点"""
        rospy.init_node('kitti_pointpillars_bag_detector', anonymous=True)
        
        # 获取ROS参数
        current_dir = os.path.dirname(os.path.abspath(__file__))
        self.current_dir = Path(current_dir)
        self.project_root = self.current_dir.resolve().parents[2]
        self.workspace_root = self.current_dir.resolve().parents[3]

        self.config_path = rospy.get_param(
            '~config_path',
            str(self.project_root / 'configs' / 'pointpillars' / 'pointpillars_hv_secfpn_8xb6-160e_kitti-3d-3class.py')
        )
        self.checkpoint_path = rospy.get_param(
            '~checkpoint_path',
            str(self.project_root / 'checkpoints' / 'pointpillars_hv_secfpn_6x8_160e_kitti-3d-3class.pth')
        )
        self.bag_path = rospy.get_param(
            '~bag_path',
            str(self.project_root / 'data' / 'kitti' / 'seq_0019_with_det.bag')
        )
        self.publish_rate = rospy.get_param('~publish_rate', 10.0)  # Hz
        self.confidence_threshold = rospy.get_param('~confidence_threshold', 0.05)
        self.frame_id = rospy.get_param('~frame_id', 'velodyne')
        self.seq = f"{int(rospy.get_param('~seq', 19)):04d}"
        dataset_param = rospy.get_param('~dataset_root', str(Path('Data_Tracking') / 'training'))
        dataset_root = Path(dataset_param)
        if not dataset_root.is_absolute():
            dataset_root = (self.workspace_root / dataset_root).resolve()
        self.dataset_root = dataset_root
        self.image_width = int(rospy.get_param('~image_width', 1242))
        self.image_height = int(rospy.get_param('~image_height', 375))
        self.enable_open3d_vis = rospy.get_param('~enable_open3d_vis', False)

        # Marker 发布状态，用于清理残留标记
        self._last_marker_count = 0
        self._last_marker_frame = self.frame_id
        
        # 类别名称和颜色映射
        self.class_names = ['Car', 'Pedestrian', 'Cyclist']
        self.wireframe_color = (0.0, 0.447, 0.741, 1.0)
        self.class_colors = {name: self.wireframe_color for name in self.class_names}

        # 加载标定信息
        self._load_calibration()
        
        # 初始化推理器
        self._init_inferencer()
        
        # 创建ROS发布器
        self.marker_pub = rospy.Publisher('/detection/bboxes_3d', MarkerArray, queue_size=10)
        self.status_pub = rospy.Publisher('/detection/status', String, queue_size=10)
        self.kitti_tracking_pub = rospy.Publisher('/detection/kitti_tracking', String, queue_size=10)
        self.pointcloud_pub = rospy.Publisher('/detection/pointcloud', PointCloud2, queue_size=10)
        self.det_pub = rospy.Publisher('/kitti/detections', Detection3DArray, queue_size=2)
        
        # 订阅点云话题
        self.pointcloud_sub = rospy.Subscriber('/kitti/velo/pointcloud', PointCloud2, self._pointcloud_callback)
        
        # 统计信息
        self.frame_count = 0
        self.last_time = rospy.Time.now()
        
        rospy.loginfo(f"KITTI PointPillars Bag ROS节点初始化完成")
        rospy.loginfo(f"配置文件: {self.config_path}")
        rospy.loginfo(f"检查点: {self.checkpoint_path}")
        rospy.loginfo(f"Bag文件: {self.bag_path}")
        rospy.loginfo(f"数据集路径: {self.dataset_root} / 序列: {self.seq}")
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

    @staticmethod
    def _empty_detections() -> Dict[str, Any]:
        return {
            'bboxes_3d': np.empty((0, 7), dtype=np.float32),
            'scores_3d': np.empty((0,), dtype=np.float32),
            'labels_3d': np.empty((0,), dtype=np.int32),
            'class_names': [],
        }

    def _load_calibration(self) -> None:
        """读取KITTI标定文件，构造LiDAR到相机的外参与投影矩阵。"""
        seq_root = self.dataset_root / "sequences" if (self.dataset_root / "sequences").is_dir() else self.dataset_root
        calib_dir = seq_root / "calib"
        if calib_dir.is_dir():
            calib_path = calib_dir / f"{self.seq}.txt"
        else:
            calib_path = seq_root / self.seq / "calib.txt"

        # 如果calib.txt不存在，尝试读取分开的标定文件
        if not calib_path.exists():
            rospy.loginfo(f"calib.txt不存在，尝试读取分开的标定文件")
            self._load_separate_calibration_files(seq_root / self.seq)
            return

        calib_data: Dict[str, np.ndarray] = {}
        with calib_path.open("r") as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                if ":" in line:
                    key, values = line.split(":", 1)
                else:
                    parts = line.split()
                    key, values = parts[0], " ".join(parts[1:])
                calib_data[key] = np.fromstring(values, sep=" ", dtype=np.float64)

        if "Tr_velo_cam" not in calib_data or "P2" not in calib_data:
            raise ValueError(f"标定文件缺少Tr_velo_cam或P2: {calib_path}")

        lidar2cam = np.vstack((calib_data["Tr_velo_cam"].reshape(3, 4), [0.0, 0.0, 0.0, 1.0]))
        cam2img = np.eye(4, dtype=np.float64)
        cam2img[:3, :4] = calib_data["P2"].reshape(3, 4)

        self.lidar2cam = lidar2cam
        self.cam2img = cam2img
        self.lidar2cam_tensor = torch.from_numpy(self.lidar2cam[:3, :4]).float()
        self.cam2img_tensor = torch.from_numpy(self.cam2img[:3, :4]).float()

        rospy.loginfo(f"加载标定文件成功: {calib_path}")

    def _load_separate_calibration_files(self, calib_dir: Path) -> None:
        """读取分开的标定文件（calib_cam_to_cam.txt, calib_velo_to_cam.txt等）"""
        # 读取相机标定文件
        cam_to_cam_path = calib_dir / "calib_cam_to_cam.txt"
        velo_to_cam_path = calib_dir / "calib_velo_to_cam.txt"
        
        if not cam_to_cam_path.exists() or not velo_to_cam_path.exists():
            raise FileNotFoundError(f"找不到标定文件: {cam_to_cam_path} 或 {velo_to_cam_path}")

        # 读取P2（相机内参矩阵）
        p2_data = None
        with cam_to_cam_path.open("r") as f:
            for line in f:
                line = line.strip()
                if line.startswith("P_rect_02:"):
                    values = line.split(":", 1)[1].strip()
                    p2_data = np.fromstring(values, sep=" ", dtype=np.float64).reshape(3, 4)
                    break
        
        if p2_data is None:
            raise ValueError(f"找不到P_rect_02在文件: {cam_to_cam_path}")

        # 读取激光雷达到相机的变换矩阵
        r_data = None
        t_data = None
        with velo_to_cam_path.open("r") as f:
            for line in f:
                line = line.strip()
                if line.startswith("R:"):
                    values = line.split(":", 1)[1].strip()
                    r_data = np.fromstring(values, sep=" ", dtype=np.float64).reshape(3, 3)
                elif line.startswith("T:"):
                    values = line.split(":", 1)[1].strip()
                    t_data = np.fromstring(values, sep=" ", dtype=np.float64)
        
        if r_data is None or t_data is None:
            raise ValueError(f"找不到R或T在文件: {velo_to_cam_path}")

        # 构造Tr_velo_cam矩阵（3x4）
        tr_velo_cam = np.hstack([r_data, t_data.reshape(3, 1)])

        # 构造变换矩阵
        lidar2cam = np.vstack((tr_velo_cam, [0.0, 0.0, 0.0, 1.0]))
        cam2img = np.eye(4, dtype=np.float64)
        cam2img[:3, :4] = p2_data

        self.lidar2cam = lidar2cam
        self.cam2img = cam2img
        self.lidar2cam_tensor = torch.from_numpy(self.lidar2cam[:3, :4]).float()
        self.cam2img_tensor = torch.from_numpy(self.cam2img[:3, :4]).float()

        rospy.loginfo(f"加载分开的标定文件成功: {cam_to_cam_path}, {velo_to_cam_path}")
    
    def _pointcloud_callback(self, msg: PointCloud2):
        """点云数据回调函数"""
        try:
            self.frame_count += 1
            frame_id = max(self.frame_count - 1, 0)
            
            # 保存原始时间戳，用于后续发布
            original_stamp = msg.header.stamp
            
            # 发布状态
            self._publish_status(f"处理帧 {frame_id}: {original_stamp}")
            
            # 转换点云数据
            points = self._pointcloud2_to_numpy(msg)
            if points is None:
                rospy.logwarn("点云转换失败")
                return
            
            # 运行推理
            detections = self._run_inference(points, original_stamp)

            det_header = Header(
                stamp=original_stamp,  # 使用原始时间戳
                frame_id=f"seq_{self.seq}",
            )
            cam_info = self._publish_detection_array(detections, det_header, frame_id)
            
            # 转换为MarkerArray并发布
            marker_frame_id = msg.header.frame_id if msg.header.frame_id else self.frame_id
            marker_array = self._detections_to_markerarray(detections, marker_frame_id, original_stamp)
            self.marker_pub.publish(marker_array)
            
            # 转换为KITTI tracking格式并发布
            if cam_info is not None:
                cam_centers, dims_kitti, cam_yaw, class_names, scores, bbox2d_list = cam_info
                kitti_tracking = self._detections_to_kitti_tracking(
                    frame_id, cam_centers, dims_kitti, cam_yaw, class_names, scores, bbox2d_list
                )
            else:
                kitti_tracking = ""
            kitti_msg = String()
            kitti_msg.data = kitti_tracking
            self.kitti_tracking_pub.publish(kitti_msg)
            
            # 发布原始点云（可选）
            self.pointcloud_pub.publish(msg)
            
            # 发布检测统计信息
            num_detections = int(detections['bboxes_3d'].shape[0])
            rospy.loginfo(f"帧 {frame_id}: 检测到 {num_detections} 个目标")
            
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
        temp_file = None
        try:
            # 保存临时点云文件
            temp_file = f"/tmp/kitti_points_{timestamp.secs}_{timestamp.nsecs}.bin"
            points.astype(np.float32).tofile(temp_file)
            
            # 运行推理
            result = self.inferencer(
                inputs=dict(points=temp_file),
                show=self.enable_open3d_vis,
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
            return self._empty_detections()
        finally:
            # 确保临时文件被清理
            if temp_file and os.path.exists(temp_file):
                try:
                    os.remove(temp_file)
                except OSError:
                    pass  # 忽略删除失败的错误

    def _publish_detection_array(
        self,
        detections: Dict[str, Any],
        header: Header,
        frame_id: int,
    ) -> Optional[Tuple[np.ndarray, np.ndarray, np.ndarray, List[str], np.ndarray, List[List[float]]]]:
        """将检测结果转换为Detection3DArray并发布，返回用于KITTI字符串的关键信息。"""
        msg = Detection3DArray()
        msg.header = header

        boxes = detections.get('bboxes_3d')
        scores = detections.get('scores_3d')
        labels = detections.get('labels_3d')

        if boxes is None or boxes.size == 0:
            self.det_pub.publish(msg)
            return None

        boxes_np = np.asarray(boxes, dtype=np.float32)
        scores_np = np.asarray(scores, dtype=np.float32)
        labels_np = np.asarray(labels, dtype=np.int32)

        box_dim = boxes_np.shape[1]
        lidar_boxes = LiDARInstance3DBoxes(torch.from_numpy(boxes_np), box_dim=box_dim)
        cam_boxes = lidar_boxes.convert_to(Box3DMode.CAM, rt_mat=self.lidar2cam_tensor, correct_yaw=True)
        cam_tensor = cam_boxes.tensor.cpu().numpy()
        corners_cam = cam_boxes.corners.cpu().numpy()

        dims_kitti = np.stack([boxes_np[:, 5], boxes_np[:, 4], boxes_np[:, 3]], axis=1)
        cam_centers = cam_tensor[:, :3]
        cam_yaw = cam_tensor[:, 6]

        class_names: List[str] = []
        bbox2d_list: List[List[float]] = []

        for idx in range(boxes_np.shape[0]):
            label_idx = int(labels_np[idx])
            if 0 <= label_idx < len(self.class_names):
                cls_name = self.class_names[label_idx]
            else:
                cls_name = f"class_{label_idx}"
            class_names.append(cls_name)

            bbox2d = self._project_corners_to_bbox2d(corners_cam[idx])
            bbox2d_list.append(bbox2d)

            det_msg = Detection3D()
            det_msg.header = Header(stamp=header.stamp, frame_id=header.frame_id)
            det_msg.frame = frame_id
            det_msg.track_id = -1
            det_msg.cls = cls_name
            det_msg.score = float(scores_np[idx])
            det_msg.bbox2d = [float(v) for v in bbox2d]
            det_msg.dimensions = [float(v) for v in dims_kitti[idx]]
            det_msg.location = cam_centers[idx].astype(np.float32).tolist()
            det_msg.rotation_y = float(cam_yaw[idx])
            msg.detections.append(det_msg)

        self.det_pub.publish(msg)
        return cam_centers, dims_kitti, cam_yaw, class_names, scores_np, bbox2d_list

    def _project_corners_to_bbox2d(self, corners_cam: np.ndarray) -> List[float]:
        """将3D包围盒角点投影到图像平面，得到2D bbox。"""
        ones = np.ones((corners_cam.shape[0], 1), dtype=np.float64)
        hom = np.hstack([corners_cam, ones])
        proj = hom @ self.cam2img[:3, :4].T
        depths = proj[:, 2]
        valid = depths > 0.1
        if not np.any(valid):
            return [0.0, 0.0, 0.0, 0.0]
        u = proj[valid, 0] / depths[valid]
        v = proj[valid, 1] / depths[valid]
        xmin = float(np.clip(np.min(u), 0, self.image_width - 1))
        xmax = float(np.clip(np.max(u), 0, self.image_width - 1))
        ymin = float(np.clip(np.min(v), 0, self.image_height - 1))
        ymax = float(np.clip(np.max(v), 0, self.image_height - 1))
        if xmax < xmin or ymax < ymin:
            return [0.0, 0.0, 0.0, 0.0]
        return [xmin, ymin, xmax, ymax]
    
    def _extract_detections(self, result) -> Dict[str, Any]:
        """从推理结果中提取检测信息"""
        detections = self._empty_detections()
        
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
                        bboxes = pred_instances.bboxes_3d.tensor.detach().cpu().numpy()
                    else:
                        bboxes = pred_instances.bboxes_3d.detach().cpu().numpy()
                elif isinstance(pred_instances, dict) and 'bboxes_3d' in pred_instances:
                    bboxes = pred_instances['bboxes_3d']
                
                # 获取置信度分数
                scores = None
                if hasattr(pred_instances, 'scores_3d') and pred_instances.scores_3d is not None:
                    scores = pred_instances.scores_3d.detach().cpu().numpy()
                elif isinstance(pred_instances, dict) and 'scores_3d' in pred_instances:
                    scores = pred_instances['scores_3d']
                
                # 获取标签
                labels = None
                if hasattr(pred_instances, 'labels_3d') and pred_instances.labels_3d is not None:
                    labels = pred_instances.labels_3d.detach().cpu().numpy()
                elif isinstance(pred_instances, dict) and 'labels_3d' in pred_instances:
                    labels = pred_instances['labels_3d']
                
                # 转换为numpy数组进行过滤
                if isinstance(scores, list):
                    scores = np.array(scores)
                if isinstance(bboxes, list):
                    bboxes = np.array(bboxes)
                if isinstance(labels, list):
                    labels = np.array(labels)
                
                if scores is not None and bboxes is not None and labels is not None:
                    scores = np.asarray(scores, dtype=np.float32)
                    bboxes = np.asarray(bboxes, dtype=np.float32)
                    labels = np.asarray(labels, dtype=np.int32)

                    if scores.ndim == 2 and scores.shape[1] == 1:
                        scores = scores.squeeze(axis=1)

                    # 过滤低置信度检测
                    valid_indices = scores > self.confidence_threshold
                    if np.any(valid_indices):
                        detections['scores_3d'] = scores[valid_indices].astype(np.float32)
                        detections['bboxes_3d'] = bboxes[valid_indices]
                        detections['labels_3d'] = labels[valid_indices].astype(np.int32)
                        detections['class_names'] = [
                            self.class_names[int(label)] if 0 <= int(label) < len(self.class_names) else f"class_{int(label)}"
                            for label in detections['labels_3d']
                        ]
        
        except Exception as e:
            rospy.logerr(f"提取检测结果失败: {e}")
            import traceback
            rospy.logerr(f"错误详情: {traceback.format_exc()}")
        
        return detections
    
    def _detections_to_markerarray(
        self,
        detections: Dict[str, Any],
        frame_id: str,
        stamp: rospy.Time,
    ) -> MarkerArray:
        """将检测结果转换为MarkerArray消息，并清理过期标记。"""
        marker_array = MarkerArray()

        current_count = 0

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
            marker.header.stamp = stamp
            marker.ns = "detection"
            marker.id = i
            marker.type = Marker.LINE_LIST
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
            
            # 设置线宽
            marker.scale.x = 0.05
            marker.scale.y = 0.0
            marker.scale.z = 0.0
            
            # 设置颜色
            color = self.class_colors.get(class_name, (1.0, 1.0, 1.0, 0.8))
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = color[3]
            
            # 设置生命周期
            marker.lifetime = rospy.Duration(0.0)  # 0 表示持久，直到被覆盖或删除
            marker.frame_locked = True
            
            # 线框的12条边
            half_l = float(length) / 2.0
            half_w = float(width) / 2.0
            half_h = float(height) / 2.0
            corners = [
                (half_l, half_w, half_h),
                (half_l, -half_w, half_h),
                (-half_l, -half_w, half_h),
                (-half_l, half_w, half_h),
                (half_l, half_w, -half_h),
                (half_l, -half_w, -half_h),
                (-half_l, -half_w, -half_h),
                (-half_l, half_w, -half_h),
            ]
            edges = [
                (0, 1), (1, 2), (2, 3), (3, 0),
                (4, 5), (5, 6), (6, 7), (7, 4),
                (0, 4), (1, 5), (2, 6), (3, 7),
            ]
            for start_idx, end_idx in edges:
                start = corners[start_idx]
                end = corners[end_idx]
                marker.points.append(Point(*start))
                marker.points.append(Point(*end))

            marker_array.markers.append(marker)
            current_count += 1

        if current_count < self._last_marker_count:
            # 删除多余的旧Marker，避免残影
            for marker_id in range(current_count, self._last_marker_count):
                delete_marker = Marker()
                delete_marker.header.frame_id = self._last_marker_frame
                delete_marker.header.stamp = stamp
                delete_marker.ns = "detection"
                delete_marker.id = marker_id
                delete_marker.action = Marker.DELETE
                marker_array.markers.append(delete_marker)

        self._last_marker_count = current_count
        self._last_marker_frame = frame_id
        
        return marker_array
    
    def _detections_to_kitti_tracking(
        self,
        frame_id: int,
        cam_centers: np.ndarray,
        dims_kitti: np.ndarray,
        cam_yaw: np.ndarray,
        class_names: List[str],
        scores: np.ndarray,
        bbox2d_list: List[List[float]],
    ) -> str:
        """将检测结果转换为KITTI tracking格式字符串。"""
        kitti_lines: List[str] = []

        for idx, cls_name in enumerate(class_names):
            h, w, l = dims_kitti[idx]
            x, y, z = cam_centers[idx]
            yaw = cam_yaw[idx]
            bbox_left, bbox_top, bbox_right, bbox_bottom = bbox2d_list[idx]

            # 视角角度 alpha（与KITTI定义一致）
            alpha = yaw - np.arctan2(x, z) if z != 0 else yaw

            kitti_line = (
                f"{frame_id} {idx} {cls_name} 0.0 0 {alpha:.6f} "
                f"{bbox_left:.6f} {bbox_top:.6f} {bbox_right:.6f} {bbox_bottom:.6f} "
                f"{h:.6f} {w:.6f} {l:.6f} {x:.6f} {y:.6f} {z:.6f} {yaw:.6f} {scores[idx]:.6f}"
            )
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
