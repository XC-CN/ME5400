#!/usr/bin/env python3
"""
KITTI PointPillars ROS节点 - Bag文件版本
订阅ROS bag文件中的点云数据，运行MMDetection3D PointPillars检测
"""

import os
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import rospy
import tf.transformations as tf_trans
import torch
from sensor_msgs import point_cloud2
from sensor_msgs.msg import Imu, PointCloud2, PointField
from geometry_msgs.msg import Point
from std_msgs.msg import Header, String
from visualization_msgs.msg import Marker, MarkerArray

# 追加 catkin_ws Python 包路径，便于直接运行脚本时找到自定义消息
CATKIN_PYTHON_PATH = Path(__file__).resolve().parents[4] / 'catkin_ws' / 'devel' / 'lib' / 'python3/dist-packages'
if CATKIN_PYTHON_PATH.exists():
    sys.path.insert(0, str(CATKIN_PYTHON_PATH))

# 自定义消息
from ME5400.msg import Detection3D, Detection3DArray

# MMDetection3D相关
from mmdet3d.apis import LidarDet3DInferencer
from mmdet3d.structures import Box3DMode, LiDARInstance3DBoxes

POINTFIELD_NUMPY_DTYPES = {
    PointField.INT8: 'i1',
    PointField.UINT8: 'u1',
    PointField.INT16: 'i2',
    PointField.UINT16: 'u2',
    PointField.INT32: 'i4',
    PointField.UINT32: 'u4',
    PointField.FLOAT32: 'f4',
    PointField.FLOAT64: 'f8',
}

POINTCLOUD_REQUIRED_FIELDS = ('x', 'y', 'z', 'intensity')


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

        # 兼容两种参数来源：
        # 1) 私有参数（历史用法） 2) 全局后备参数（用于匿名节点时的稳定覆盖）
        seq_default = int(rospy.get_param('/me5400_seq', 19))
        self.seq = f"{int(rospy.get_param('~seq', seq_default)):04d}"

        dataset_default = rospy.get_param('/me5400_dataset_root', str(Path('Data_Tracking') / 'training'))
        dataset_param = rospy.get_param('~dataset_root', dataset_default)
        dataset_root = Path(dataset_param)
        if not dataset_root.is_absolute():
            dataset_root = (self.workspace_root / dataset_root).resolve()
        self.dataset_root = dataset_root
        self.image_width = int(rospy.get_param('~image_width', 1242))
        self.image_height = int(rospy.get_param('~image_height', 375))
        self.enable_open3d_vis = rospy.get_param('~enable_open3d_vis', False)
        self.enable_timing_log = rospy.get_param('~enable_timing_log', True)
        self.timing_log_interval = max(1, int(rospy.get_param('~timing_log_interval', 20)))
        self.slow_frame_threshold = float(rospy.get_param('~slow_frame_threshold', 0.1))

        # Marker 发布状态，用于清理残留标记
        self._last_marker_count = 0
        self._last_marker_frame = self.frame_id
        self._timing_window_count = 0
        self._timing_window_sum: Dict[str, float] = {}
        self._pointcloud_dtype_signature: Optional[Tuple[bool, int, Tuple[Tuple[str, int, int, int], ...]]] = None
        self._pointcloud_dtype: Optional[np.dtype] = None
        
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
        self.lidar_tracking_pub = rospy.Publisher('/detection/lidar_tracking', String, queue_size=10)  # 新增：LiDAR坐标系
        self.lidar_detection_pub = rospy.Publisher('/detection/lidar_detections', Detection3DArray, queue_size=10)
        self.pointcloud_pub = rospy.Publisher('/detection/pointcloud', PointCloud2, queue_size=10)
        self.imu_pub = rospy.Publisher('/detection/imu', Imu, queue_size=10)
        
        # 订阅点云话题
        self.pointcloud_sub = rospy.Subscriber('/kitti/velo/pointcloud', PointCloud2, self._pointcloud_callback, queue_size=10)
        self.imu_sub = rospy.Subscriber('/kitti/oxts/imu', Imu, self._imu_callback, queue_size=1)
        
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
        start_time = time.perf_counter()
        try:
            self.frame_count += 1
            frame_id = max(self.frame_count - 1, 0)
            stage_timings: Dict[str, float] = {}
            
            # 保存原始时间戳，用于后续发布
            original_stamp = msg.header.stamp
            
            # 发布状态
            self._publish_status(f"处理帧 {frame_id}: {original_stamp}")
            
            # 转换点云数据
            pointcloud_start = time.perf_counter()
            points = self._pointcloud2_to_numpy(msg)
            stage_timings['pointcloud_to_numpy'] = time.perf_counter() - pointcloud_start
            if points is None:
                rospy.logwarn("点云转换失败")
                return
            
            # 运行推理
            detections, inference_timings = self._run_inference(points, original_stamp)
            stage_timings.update(inference_timings)

            det_header = Header(
                stamp=original_stamp,  # 使用原始时间戳
                frame_id=f"seq_{self.seq}",
            )
            cam_info_start = time.perf_counter()
            cam_info = self._publish_detection_array(detections, det_header, frame_id)
            stage_timings['camera_projection'] = time.perf_counter() - cam_info_start

            lidar_header = Header(
                stamp=original_stamp,
                frame_id=msg.header.frame_id if msg.header.frame_id else self.frame_id,
            )
            lidar_array_start = time.perf_counter()
            lidar_array = self._detections_to_lidar_array(detections, lidar_header, frame_id)
            self.lidar_detection_pub.publish(lidar_array)
            stage_timings['publish_lidar_detections'] = time.perf_counter() - lidar_array_start
            
            # 转换为MarkerArray并发布
            marker_start = time.perf_counter()
            marker_frame_id = msg.header.frame_id if msg.header.frame_id else self.frame_id
            marker_array = self._detections_to_markerarray(detections, marker_frame_id, original_stamp)
            self.marker_pub.publish(marker_array)
            stage_timings['publish_markers'] = time.perf_counter() - marker_start
            
            # 转换为KITTI tracking格式并发布（相机坐标系，保留用于兼容）
            kitti_start = time.perf_counter()
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
            stage_timings['publish_kitti_tracking'] = time.perf_counter() - kitti_start
            
            # 直接发布LiDAR坐标系检测结果（无需相机标定）
            lidar_tracking_start = time.perf_counter()
            lidar_tracking = self._detections_to_lidar_tracking(frame_id, detections)
            lidar_msg = String()
            lidar_msg.data = lidar_tracking
            self.lidar_tracking_pub.publish(lidar_msg)
            stage_timings['publish_lidar_tracking'] = time.perf_counter() - lidar_tracking_start
            
            # 发布原始点云（可选）
            pointcloud_pub_start = time.perf_counter()
            self.pointcloud_pub.publish(msg)
            stage_timings['republish_pointcloud'] = time.perf_counter() - pointcloud_pub_start
            
            # 发布检测统计信息
            num_detections = int(detections['bboxes_3d'].shape[0])
            rospy.loginfo(f"帧 {frame_id}: 检测到 {num_detections} 个目标")
            stage_timings['compute_total'] = time.perf_counter() - start_time
            
            # 频率控制
            current_time = rospy.Time.now()
            dt = (current_time - self.last_time).to_sec()
            if dt < 0.0:
                rospy.logwarn("检测到仿真时间回退，重置发布节流器状态")
                dt = 0.0
            if 0.0 < dt < 1.0 / self.publish_rate:
                sleep_start = time.perf_counter()
                rospy.sleep(1.0 / self.publish_rate - dt)
                stage_timings['rate_control_sleep'] = time.perf_counter() - sleep_start
            self.last_time = current_time
            
            stage_timings['wall_total'] = time.perf_counter() - start_time
            self._record_timing(frame_id, stage_timings)

        except Exception as e:
            rospy.logerr(f"点云处理失败: {e}")
            import traceback
            rospy.logerr(f"错误详情: {traceback.format_exc()}")
    
    def _pointcloud2_to_numpy(self, msg: PointCloud2) -> Optional[np.ndarray]:
        """将PointCloud2消息转换为numpy数组"""
        try:
            points = self._pointcloud2_to_numpy_fast(msg)
            if points is None:
                points = self._pointcloud2_to_numpy_fallback(msg)

            if points is None or points.size == 0:
                rospy.logwarn("点云为空")
                return None

            rospy.loginfo(f"转换得到 {len(points)} 个点")
            return points

        except Exception as e:
            rospy.logerr(f"点云转换失败: {e}")
            return None

    def _pointcloud2_to_numpy_fast(self, msg: PointCloud2) -> Optional[np.ndarray]:
        """使用numpy直接解析PointCloud2二进制buffer，避免逐点Python循环。"""
        structured_dtype = self._get_pointcloud_structured_dtype(msg)
        if structured_dtype is None:
            return None

        num_points = msg.width * msg.height
        if num_points == 0:
            return None

        if msg.height == 1 or msg.row_step == msg.point_step * msg.width:
            structured = np.frombuffer(msg.data, dtype=structured_dtype, count=num_points)
        else:
            structured = np.ndarray(
                shape=(msg.height, msg.width),
                dtype=structured_dtype,
                buffer=msg.data,
                strides=(msg.row_step, msg.point_step),
            ).reshape(-1)

        x = structured['x']
        y = structured['y']
        z = structured['z']
        intensity = structured['intensity']
        valid_mask = np.isfinite(x) & np.isfinite(y) & np.isfinite(z) & np.isfinite(intensity)
        valid_count = int(np.count_nonzero(valid_mask))
        if valid_count == 0:
            return None

        points = np.empty((valid_count, 4), dtype=np.float32)
        if valid_count == num_points:
            points[:, 0] = x
            points[:, 1] = y
            points[:, 2] = z
            points[:, 3] = intensity
            return points

        points[:, 0] = x[valid_mask]
        points[:, 1] = y[valid_mask]
        points[:, 2] = z[valid_mask]
        points[:, 3] = intensity[valid_mask]
        return points

    def _pointcloud2_to_numpy_fallback(self, msg: PointCloud2) -> Optional[np.ndarray]:
        """字段布局不满足快速路径时，回退到兼容的逐点解析实现。"""
        points_list = []
        for data in point_cloud2.read_points(msg, field_names=POINTCLOUD_REQUIRED_FIELDS, skip_nans=True):
            points_list.append([data[0], data[1], data[2], data[3]])

        if not points_list:
            return None

        return np.asarray(points_list, dtype=np.float32)

    def _get_pointcloud_structured_dtype(self, msg: PointCloud2) -> Optional[np.dtype]:
        signature = (
            msg.is_bigendian,
            msg.point_step,
            tuple((field.name, field.offset, field.datatype, field.count) for field in msg.fields),
        )
        if signature == self._pointcloud_dtype_signature:
            return self._pointcloud_dtype

        endianness = '>' if msg.is_bigendian else '<'
        available_fields = {field.name: field for field in msg.fields}

        names: List[str] = []
        formats: List[np.dtype] = []
        offsets: List[int] = []
        for field_name in POINTCLOUD_REQUIRED_FIELDS:
            field = available_fields.get(field_name)
            if field is None:
                self._pointcloud_dtype_signature = signature
                self._pointcloud_dtype = None
                rospy.logwarn_once(
                    "PointCloud2缺少字段%s，回退到逐点解析",
                    field_name,
                )
                return None

            if field.count != 1:
                self._pointcloud_dtype_signature = signature
                self._pointcloud_dtype = None
                rospy.logwarn_once(
                    "PointCloud2字段%s count=%d，回退到逐点解析",
                    field_name,
                    field.count,
                )
                return None

            numpy_code = POINTFIELD_NUMPY_DTYPES.get(field.datatype)
            if numpy_code is None:
                self._pointcloud_dtype_signature = signature
                self._pointcloud_dtype = None
                rospy.logwarn_once(
                    "PointCloud2字段%s datatype=%d不支持，回退到逐点解析",
                    field_name,
                    field.datatype,
                )
                return None

            names.append(field.name)
            formats.append(np.dtype(f'{endianness}{numpy_code}'))
            offsets.append(field.offset)

        self._pointcloud_dtype_signature = signature
        self._pointcloud_dtype = np.dtype(
            {
                'names': names,
                'formats': formats,
                'offsets': offsets,
                'itemsize': msg.point_step,
            }
        )
        return self._pointcloud_dtype
    
    def _run_inference(self, points: np.ndarray, timestamp) -> Tuple[Dict[str, Any], Dict[str, float]]:
        """运行推理"""
        temp_file = None
        timings: Dict[str, float] = {}
        try:
            save_start = time.perf_counter()
            temp_file = f"/tmp/kitti_points_{timestamp.secs}_{timestamp.nsecs}.bin"
            points.astype(np.float32).tofile(temp_file)
            timings['write_temp_bin'] = time.perf_counter() - save_start

            infer_start = time.perf_counter()
            result = self.inferencer(
                inputs=dict(points=temp_file),
                show=self.enable_open3d_vis,
                wait_time=0,
                pred_score_thr=self.confidence_threshold,
                no_save_vis=True,
                no_save_pred=True
            )
            timings['inferencer_call'] = time.perf_counter() - infer_start

            extract_start = time.perf_counter()
            detections = self._extract_detections(result)
            timings['extract_detections'] = time.perf_counter() - extract_start
            return detections, timings
            
        except Exception as e:
            rospy.logerr(f"推理失败: {e}")
            return self._empty_detections(), timings
        finally:
            if temp_file and os.path.exists(temp_file):
                cleanup_start = time.perf_counter()
                try:
                    os.remove(temp_file)
                except OSError:
                    pass  # 忽略删除失败的错误
                timings['remove_temp_bin'] = time.perf_counter() - cleanup_start

    def _publish_detection_array(
        self,
        detections: Dict[str, Any],
        header: Header,
        frame_id: int,
    ) -> Optional[Tuple[np.ndarray, np.ndarray, np.ndarray, List[str], np.ndarray, List[List[float]]]]:
        """将检测结果转换为KITTI格式所需的关键信息。"""
        boxes = detections.get('bboxes_3d')
        scores = detections.get('scores_3d')
        labels = detections.get('labels_3d')

        if boxes is None or boxes.size == 0:
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

    def _detections_to_lidar_array(
        self,
        detections: Dict[str, Any],
        header: Header,
        frame_id: int,
    ) -> Detection3DArray:
        array_msg = Detection3DArray()
        array_msg.header = header

        bboxes = detections.get('bboxes_3d')
        scores = detections.get('scores_3d')
        class_names = detections.get('class_names')

        if bboxes is None or bboxes.size == 0:
            return array_msg

        for bbox, score, cls_name in zip(bboxes, scores, class_names):
            if len(bbox) == 7:
                x, y, z, length, width, height, yaw = bbox
            elif len(bbox) == 9:
                x, y, z, length, width, height, yaw, _vx, _vy = bbox
            else:
                continue

            det_msg = Detection3D()
            det_msg.header = header
            det_msg.frame = int(frame_id)
            det_msg.track_id = -1
            det_msg.cls = cls_name
            det_msg.score = float(score)
            det_msg.location = [float(x), float(y), float(z)]
            det_msg.dimensions = [float(height), float(width), float(length)]
            det_msg.rotation_y = float(yaw)
            det_msg.bbox2d = [0.0, 0.0, 0.0, 0.0]
            array_msg.detections.append(det_msg)

        return array_msg
    
    def _detections_to_lidar_tracking(
        self,
        frame_id: int,
        detections: Dict[str, Any],
    ) -> str:
        """
        将检测结果转换为 LiDAR 坐标系 tracking 格式字符串
        格式：frame track_id class score x y z l w h yaw
        无需相机标定，直接使用 LiDAR 坐标系的检测结果
        """
        boxes = detections.get('bboxes_3d')
        scores = detections.get('scores_3d')
        labels = detections.get('labels_3d')
        class_names = detections.get('class_names', [])
        
        if boxes is None or boxes.size == 0:
            return ""
        
        boxes_np = np.asarray(boxes, dtype=np.float32)
        scores_np = np.asarray(scores, dtype=np.float32)
        
        lidar_lines: List[str] = []
        for idx in range(boxes_np.shape[0]):
            # LiDAR 坐标系：boxes_np[idx] = [x, y, z, l, w, h, yaw]
            x, y, z, l, w, h, yaw = boxes_np[idx]
            score = scores_np[idx]
            cls_name = class_names[idx] if idx < len(class_names) else "Unknown"
            
            # 格式：frame track_id class score x y z l w h yaw
            lidar_line = (
                f"{frame_id} {idx} {cls_name} {score:.6f} "
                f"{x:.6f} {y:.6f} {z:.6f} {l:.6f} {w:.6f} {h:.6f} {yaw:.6f}"
            )
            lidar_lines.append(lidar_line)
        
        return '\n'.join(lidar_lines)
    
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
        """将检测结果转换为KITTI tracking格式字符串（相机坐标系，需要标定）。"""
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

    def _imu_callback(self, msg: Imu) -> None:
        """转发IMU数据"""
        self.imu_pub.publish(msg)

    def _record_timing(self, frame_id: int, stage_timings: Dict[str, float]) -> None:
        if not self.enable_timing_log:
            return

        compute_total = stage_timings.get('compute_total', 0.0)
        if compute_total > self.slow_frame_threshold:
            ordered = sorted(
                ((name, duration) for name, duration in stage_timings.items() if name not in {'compute_total', 'wall_total'}),
                key=lambda item: item[1],
                reverse=True,
            )
            breakdown = ', '.join(
                f"{name}={duration * 1000.0:.1f}ms" for name, duration in ordered if duration > 0.0
            )
            rospy.logwarn(
                "慢帧 %d: 纯计算耗时 %.1fms，分段=%s",
                frame_id,
                compute_total * 1000.0,
                breakdown,
            )

        self._timing_window_count += 1
        for key, value in stage_timings.items():
            self._timing_window_sum[key] = self._timing_window_sum.get(key, 0.0) + value

        if self._timing_window_count < self.timing_log_interval:
            return

        count = float(self._timing_window_count)
        ordered_keys = [
            'pointcloud_to_numpy',
            'write_temp_bin',
            'inferencer_call',
            'extract_detections',
            'remove_temp_bin',
            'camera_projection',
            'publish_lidar_detections',
            'publish_markers',
            'publish_kitti_tracking',
            'publish_lidar_tracking',
            'republish_pointcloud',
            'compute_total',
            'rate_control_sleep',
            'wall_total',
        ]
        label_map = {
            'pointcloud_to_numpy': '点云解析',
            'write_temp_bin': '临时落盘',
            'inferencer_call': '模型推理',
            'extract_detections': '结果提取',
            'remove_temp_bin': '清理临时文件',
            'camera_projection': '相机坐标转换',
            'publish_lidar_detections': '发布Detection3DArray',
            'publish_markers': '发布MarkerArray',
            'publish_kitti_tracking': '发布KITTI串',
            'publish_lidar_tracking': '发布LiDAR串',
            'republish_pointcloud': '转发点云',
            'compute_total': '纯计算总耗时',
            'rate_control_sleep': '限速休眠',
            'wall_total': '墙钟总耗时',
        }
        timing_parts = []
        seen = set()
        for key in ordered_keys:
            if key not in self._timing_window_sum:
                continue
            timing_parts.append(
                f"{label_map[key]}={1000.0 * self._timing_window_sum[key] / count:.1f}ms"
            )
            seen.add(key)
        for key in sorted(self._timing_window_sum.keys()):
            if key in seen:
                continue
            timing_parts.append(f"{key}={1000.0 * self._timing_window_sum[key] / count:.1f}ms")

        rospy.loginfo(
            "PointPillars计时汇总(%d帧均值): %s",
            self._timing_window_count,
            ', '.join(timing_parts),
        )
        self._timing_window_count = 0
        self._timing_window_sum = {}


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
