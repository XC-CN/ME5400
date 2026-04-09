#!/usr/bin/env python3
"""Online MCTrack node consuming FAST-LIO pose and PointPillars detections."""
from __future__ import annotations

import math
import sys
import threading
import time
from collections import defaultdict, deque
from pathlib import Path
from typing import Deque, Dict, List, Optional, Tuple

import numpy as np
import rospy
import tf.transformations as tf_trans
import yaml
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Header, String
from visualization_msgs.msg import Marker, MarkerArray

PACKAGE_DIR = Path(__file__).resolve()
REPO_ROOT = PACKAGE_DIR.parents[4]
MCTrack_DIR = REPO_ROOT / "MCTrack"
SCRIPTS_DIR = REPO_ROOT / "Scripts"
if str(MCTrack_DIR) not in sys.path:
    sys.path.append(str(MCTrack_DIR))
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.append(str(SCRIPTS_DIR))

from tracker.frame import Frame  # noqa: E402
from tracker.bbox import BBox  # noqa: E402
from tracker.base_tracker import Base3DTracker  # noqa: E402
from ME5400.msg import Detection3D, Detection3DArray, TrackedObject, TrackedObjectArray  # noqa: E402


def read_calib(calib_path: Path) -> Dict[str, np.ndarray]:
    data: Dict[str, np.ndarray] = {}
    with calib_path.open("r") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            if ":" in line:
                key, values = line.split(":", 1)
            else:
                parts = line.split()
                if not parts:
                    continue
                key, values = parts[0], " ".join(parts[1:])
            arr = np.fromstring(values, sep=" ", dtype=np.float64)
            data[key] = arr
    return data


def build_camera_info(lidar2camera: np.ndarray, camera2image: np.ndarray) -> Dict[str, Dict]:
    lidar2camera_list = lidar2camera.tolist()
    camera2image_list = camera2image.tolist()
    cam_dict = {
        "image_shape": (1242, 375),
        "ego2camera": None,
        "camera2image": camera2image_list,
        "lidar2camera": lidar2camera_list,
        "camera_token": None,
        "camera_image": None,
    }
    return {"CAM_FRONT": cam_dict}


def quaternion_from_yaw(yaw: float) -> np.ndarray:
    half = yaw / 2.0
    return np.array([math.cos(half), 0.0, 0.0, math.sin(half)], dtype=np.float64)


def quaternion_to_matrix(quat: np.ndarray) -> np.ndarray:
    w, x, y, z = quat
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def pose_to_matrix(msg: PoseStamped) -> np.ndarray:
    q = msg.pose.orientation
    quat = np.array([q.w, q.x, q.y, q.z])
    rot = quaternion_to_matrix(quat)
    T = np.eye(4)
    T[:3, :3] = rot
    T[0, 3] = msg.pose.position.x
    T[1, 3] = msg.pose.position.y
    T[2, 3] = msg.pose.position.z
    return T


def interpolate_pose_matrix(T0: np.ndarray, T1: np.ndarray, alpha: float) -> np.ndarray:
    alpha = max(0.0, min(1.0, alpha))
    q0 = tf_trans.quaternion_from_matrix(T0)
    q1 = tf_trans.quaternion_from_matrix(T1)
    q = tf_trans.quaternion_slerp(q0, q1, alpha)
    T = tf_trans.quaternion_matrix(q)
    T[:3, 3] = (1.0 - alpha) * T0[:3, 3] + alpha * T1[:3, 3]
    return T


def camera_to_lidar(camera_xyz: np.ndarray, rot_y: float, camera2lidar: np.ndarray) -> np.ndarray:
    hom = np.concatenate([camera_xyz, [1.0]])
    lidar_xyz = camera2lidar @ hom
    return lidar_xyz[:3]


def project_global(lidar_xyz: np.ndarray, lidar2global: np.ndarray) -> np.ndarray:
    hom = np.concatenate([lidar_xyz, [1.0]])
    global_xyz = lidar2global @ hom
    return global_xyz[:3]


def angle_wrap(angle: float) -> float:
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def resolve_dataset_root(dataset_param: Optional[str]) -> Path:
    candidates: List[Path] = []

    if dataset_param:
        dataset_root = Path(dataset_param)
        if not dataset_root.is_absolute():
            dataset_root = (REPO_ROOT / dataset_root).resolve()
        candidates.append(dataset_root)

    default_roots = [
        REPO_ROOT / "tracking" / "training",
        REPO_ROOT / "Data_Tracking" / "training",
    ]
    for root in default_roots:
        resolved = root.resolve()
        if resolved not in candidates:
            candidates.append(resolved)

    for candidate in candidates:
        if candidate.exists():
            return candidate

    checked = ", ".join(str(path) for path in candidates) or "<none>"
    raise FileNotFoundError(f"Unable to locate KITTI dataset. Checked: {checked}")


def get_global_yaw(rot_y: float, lidar2global: np.ndarray) -> float:
    """从相机坐标系yaw转换到全局坐标系yaw"""
    cos_theta = lidar2global[0, 0]
    sin_theta = lidar2global[1, 0]
    cos_theta = max(-1.0, min(1.0, cos_theta))
    theta = math.acos(cos_theta)
    if sin_theta < 0:
        theta = 2 * math.pi - theta
    new_yaw = (math.pi - rot_y) + math.pi / 2.0
    return angle_wrap(new_yaw + theta)


def get_global_yaw_from_lidar(lidar_yaw: float, lidar2global: np.ndarray) -> float:
    """从LiDAR坐标系yaw转换到全局坐标系yaw"""
    cos_theta = lidar2global[0, 0]
    sin_theta = lidar2global[1, 0]
    cos_theta = max(-1.0, min(1.0, cos_theta))
    theta = math.acos(cos_theta)
    if sin_theta < 0:
        theta = 2 * math.pi - theta
    # LiDAR坐标系：直接加上旋转角
    return angle_wrap(lidar_yaw + theta)


class MCTrackOnlineNode:
    def __init__(self) -> None:
        config_path = Path(rospy.get_param("~config", MCTrack_DIR / "config" / "kitti_fastlio.yaml"))
        self.pose_topic = rospy.get_param("~pose_topic", "/mctrack/lidar_pose")
        self.use_string_det = bool(rospy.get_param("~use_lidar_tracking_string", False))
        # 默认使用带时间戳的Detection3DArray（LiDAR坐标系）
        if self.use_string_det:
            self.det_topic = rospy.get_param("~det_topic", "/detection/lidar_tracking")
        else:
            self.det_topic = rospy.get_param("~det_topic", "/detection/lidar_detections")
        self.frame_rate = float(rospy.get_param("~frame_rate", 10.0))
        self.arrow_length = float(rospy.get_param("~arrow_length", 3.0))
        self.history_size = int(rospy.get_param("~history_size", 200))
        self.pose_buffer_size = int(rospy.get_param("~pose_buffer_size", 256))
        self.pending_detection_limit = int(rospy.get_param("~pending_detection_limit", 100))
        self.pose_wait_timeout = float(rospy.get_param("~pose_wait_timeout", 0.25))

        with config_path.open("r") as f:
            self.cfg = yaml.safe_load(f)

        self.tracker = Base3DTracker(self.cfg)
        
        # PointPillars直接输出LiDAR坐标系，无需相机标定
        # MCTrack只需要知道检测框在LiDAR坐标系即可
        self.cameras_transform_matrix = {"CAM_FRONT": {
            "lidar2camera": None,
            "camera2image": None,
            "camera_token": None,
            "camera_image": None,
        }}

        self.pose_matrix: Optional[np.ndarray] = None
        self.pose_buffer: Deque[Tuple[float, np.ndarray]] = deque(maxlen=self.pose_buffer_size)
        self.pending_detections: Deque[Tuple[Detection3DArray, float]] = deque()
        self._drain_lock = threading.Lock()
        self.traj_history: Dict[int, List[np.ndarray]] = defaultdict(list)

        self.marker_pub = rospy.Publisher("/mctrack/markers", MarkerArray, queue_size=1)
        self.track_pub = rospy.Publisher("/mctrack/tracked_objects", TrackedObjectArray, queue_size=5)
        self.pose_sub = rospy.Subscriber(self.pose_topic, PoseStamped, self.pose_callback, queue_size=20)
        if self.use_string_det:
            self.det_sub = rospy.Subscriber(self.det_topic, String, self.lidar_tracking_callback, queue_size=10)
        else:
            self.det_sub = rospy.Subscriber(self.det_topic, Detection3DArray, self.det_callback, queue_size=10)
        
        rospy.loginfo(
            "MCTrackOnlineNode 初始化完成 (配置=%s, 检测话题=%s, 旧版String=%s)",
            config_path,
            self.det_topic,
            self.use_string_det,
        )

    def pose_callback(self, msg: PoseStamped) -> None:
        self.pose_matrix = pose_to_matrix(msg)
        pose_stamp = msg.header.stamp.to_sec() if msg.header.stamp != rospy.Time() else rospy.Time.now().to_sec()
        self.pose_buffer.append((float(pose_stamp), self.pose_matrix.copy()))
        self._drain_pending_detections()

    def lidar_tracking_callback(self, msg: String) -> None:
        """处理LiDAR坐标系检测结果"""
        detection_msg = self._convert_lidar_tracking_to_array(msg)
        if detection_msg is not None:
            self.det_callback(detection_msg)

    def det_callback(self, msg: Detection3DArray) -> None:
        if self.pose_matrix is None and not self.pose_buffer:
            rospy.logwarn_throttle(5.0, "尚未收到 FAST-LIO 位姿，暂时缓存检测结果")
        if len(self.pending_detections) >= self.pending_detection_limit:
            self.pending_detections.popleft()
            rospy.logwarn_throttle(5.0, "待处理检测结果积压过多，已丢弃最旧的一帧")
        self.pending_detections.append((msg, time.monotonic()))
        self._drain_pending_detections()

    def _drain_pending_detections(self) -> None:
        if not self._drain_lock.acquire(blocking=False):
            return
        try:
            while self.pending_detections:
                msg, enqueue_time = self.pending_detections[0]
                lidar2global = self._resolve_pose_for_detection(msg, enqueue_time)
                if lidar2global is None:
                    break
                self.pending_detections.popleft()
                self._process_detection(msg, lidar2global)
        finally:
            self._drain_lock.release()

    def _resolve_pose_for_detection(
        self,
        msg: Detection3DArray,
        enqueue_time: float,
    ) -> Optional[np.ndarray]:
        if not self.pose_buffer:
            return None

        det_stamp = msg.header.stamp.to_sec() if msg.header.stamp != rospy.Time() else float("nan")
        if not math.isfinite(det_stamp):
            return self.pose_buffer[-1][1]

        prev_pose: Optional[Tuple[float, np.ndarray]] = None
        next_pose: Optional[Tuple[float, np.ndarray]] = None
        for stamp, matrix in self.pose_buffer:
            if stamp <= det_stamp:
                prev_pose = (stamp, matrix)
            if stamp >= det_stamp:
                next_pose = (stamp, matrix)
                break

        if prev_pose is not None and next_pose is not None:
            prev_stamp, prev_matrix = prev_pose
            next_stamp, next_matrix = next_pose
            if next_stamp <= prev_stamp:
                return next_matrix
            alpha = (det_stamp - prev_stamp) / (next_stamp - prev_stamp)
            return interpolate_pose_matrix(prev_matrix, next_matrix, alpha)

        if (time.monotonic() - enqueue_time) < self.pose_wait_timeout:
            return None

        if next_pose is not None:
            return next_pose[1]
        if prev_pose is not None:
            rospy.logwarn_throttle(5.0, "等待与检测时间戳对齐的位姿超时，回退使用最新 FAST-LIO 位姿")
            return prev_pose[1]
        return None

    def _process_detection(self, msg: Detection3DArray, lidar2global: np.ndarray) -> None:
        frame_id = 0
        if msg.detections:
            frame_id = msg.detections[0].frame
        header = msg.header if msg.header.stamp != rospy.Time() else Header(stamp=rospy.Time.now())
        global2lidar = np.linalg.inv(lidar2global)

        frame = Frame(frame_id=frame_id, timestamp=header.stamp.to_sec(), transform_matrix={
            "global2ego": None,
            "ego2lidar": None,
            "global2lidar": global2lidar.tolist(),
            "cameras_transform_matrix": self.cameras_transform_matrix,
        })

        online_scores = self.cfg["THRESHOLD"]["INPUT_SCORE"]["ONLINE"]
        cat_map = self.cfg["CATEGORY_MAP_TO_NUMBER"]

        for det in msg.detections:
            cls = det.cls.lower()
            # 分数阈值放宽到0.2（支持所有类别：car, pedestrian, cyclist）
            if det.score < 0.2:
                continue
            dims = np.array(det.dimensions, dtype=np.float64)
            lwh = [dims[2], dims[1], dims[0]]
            
            # 检测框已经在LiDAR坐标系，直接转换到全局坐标系
            lidar_xyz = np.array(det.location, dtype=np.float64)
            global_xyz = project_global(lidar_xyz, lidar2global)
            lidar_yaw = det.rotation_y
            global_yaw = get_global_yaw_from_lidar(lidar_yaw, lidar2global)
            global_orientation = quaternion_from_yaw(global_yaw)

            bbox = {
                "detection_score": float(det.score),
                "category": cls,
                "global_xyz": global_xyz.tolist(),
                "global_orientation": global_orientation.tolist(),
                "global_yaw": float(global_yaw),
                "lwh": lwh,
                "global_velocity": [0.0, 0.0],
                "global_acceleration": [0.0, 0.0],
                "lidar_xyz": lidar_xyz.tolist(),
                "lidar_orientation": quaternion_from_yaw(lidar_yaw),
                "lidar_yaw": float(lidar_yaw),
                "bbox_image": {
                    "camera_type": "CAM_FRONT",
                    "x1y1x2y2": list(map(float, det.bbox2d)),
                },
            }
            frame.bboxes.append(BBox(frame_id, bbox))

        if not frame.bboxes:
            return

        outputs = self.tracker.track_single_frame(frame)
        self.publish_tracks(outputs, header)
        self.publish_markers(outputs, header)

    def _convert_lidar_tracking_to_array(self, msg: String) -> Optional[Detection3DArray]:
        """
        将LiDAR坐标系tracking格式转换为Detection3DArray
        格式：frame track_id class score x y z l w h yaw
        """
        data = msg.data.strip()
        if not data:
            empty_msg = Detection3DArray()
            empty_msg.header = Header(stamp=rospy.Time.now(), frame_id="velodyne")
            return empty_msg

        lines = [line.strip() for line in data.splitlines() if line.strip()]
        stamp = rospy.Time.now()
        array_msg = Detection3DArray()
        array_msg.header = Header(stamp=stamp, frame_id="velodyne")

        for line in lines:
            # LiDAR tracking format: frame track_id class score x y z l w h yaw
            parts = line.split()
            if len(parts) < 11:
                rospy.logwarn_throttle(
                    10.0,
                    "忽略字段数量不足的 LiDAR tracking 行 (%d 项): %s",
                    len(parts),
                    line,
                )
                continue
            try:
                det_msg = Detection3D()
                det_msg.header = Header(stamp=stamp, frame_id=array_msg.header.frame_id)
                det_msg.frame = int(float(parts[0]))
                det_msg.track_id = int(float(parts[1]))
                det_msg.cls = parts[2]
                det_msg.score = float(parts[3])
                # LiDAR坐标系：直接使用xyz作为location
                x, y, z = float(parts[4]), float(parts[5]), float(parts[6])
                l, w, h = float(parts[7]), float(parts[8]), float(parts[9])
                yaw = float(parts[10])
                
                # 存储为Detection3D消息（已经在LiDAR坐标系）
                det_msg.location = [x, y, z]
                det_msg.dimensions = [h, w, l]  # KITTI格式是h,w,l
                det_msg.rotation_y = yaw
                det_msg.bbox2d = [0.0, 0.0, 0.0, 0.0]  # LiDAR检测无2D bbox
                array_msg.detections.append(det_msg)
            except (ValueError, IndexError) as exc:
                rospy.logwarn_throttle(10.0, "解析 LiDAR tracking 行 '%s' 失败: %s", line, exc)
        return array_msg

    def _convert_kitti_tracking_to_array(self, msg: String) -> Optional[Detection3DArray]:
        """已弃用：保留用于向后兼容"""
        data = msg.data.strip()
        if not data:
            empty_msg = Detection3DArray()
            empty_msg.header = Header(stamp=rospy.Time.now(), frame_id="velodyne")
            return empty_msg

        lines = [line.strip() for line in data.splitlines() if line.strip()]
        stamp = rospy.Time.now()
        array_msg = Detection3DArray()
        array_msg.header = Header(stamp=stamp, frame_id="velodyne")

        for line in lines:
            # KITTI tracking format: frame track_id type truncation occlusion alpha bbox_left bbox_top bbox_right bbox_bottom h w l x y z yaw score
            parts = line.split()
            if len(parts) < 18:
                rospy.logwarn_throttle(
                    10.0,
                    "忽略字段数量不足的 KITTI 行 (%d 项): %s",
                    len(parts),
                    line,
                )
                continue
            try:
                det_msg = Detection3D()
                det_msg.header = Header(stamp=stamp, frame_id=array_msg.header.frame_id)
                det_msg.frame = int(float(parts[0]))
                det_msg.track_id = -1
                det_msg.cls = parts[2]
                det_msg.score = float(parts[17])
                det_msg.bbox2d = [float(parts[i]) for i in range(6, 10)]
                det_msg.dimensions = [float(parts[i]) for i in range(10, 13)]
                det_msg.location = [float(parts[i]) for i in range(13, 16)]
                det_msg.rotation_y = float(parts[16])
                array_msg.detections.append(det_msg)
            except (ValueError, IndexError) as exc:
                rospy.logwarn_throttle(10.0, "解析 KITTI 行 '%s' 失败: %s", line, exc)
        return array_msg

    def publish_markers(self, outputs: Dict[int, BBox], header: Header) -> None:
        markers: List[Marker] = []
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.append(clear)
        stamp = header.stamp if header.stamp != rospy.Time() else rospy.Time.now()
        frame_id_base = "camera_init"
        for track_id, bbox in outputs.items():
            pos = bbox.global_xyz
            yaw = bbox.global_yaw
            length, width, height = bbox.lwh

            history = self.traj_history[track_id]
            history.append(np.array(pos))
            if len(history) > self.history_size:
                history.pop(0)

            cube = Marker()
            cube.header.stamp = stamp
            cube.header.frame_id = frame_id_base
            cube.ns = "mctrack_bbox"
            cube.id = track_id * 10
            cube.type = Marker.CUBE
            cube.action = Marker.ADD
            cube.pose.position.x, cube.pose.position.y, cube.pose.position.z = pos
            q = quaternion_from_yaw(yaw)
            cube.pose.orientation.x = q[1]
            cube.pose.orientation.y = q[2]
            cube.pose.orientation.z = q[3]
            cube.pose.orientation.w = q[0]
            cube.scale.x = length
            cube.scale.y = width
            cube.scale.z = height
            hue = (track_id * 53) % 360
            r, g, b = self.hsv_to_rgb(hue / 360.0, 0.9, 0.9)
            cube.color.r = r
            cube.color.g = g
            cube.color.b = b
            cube.color.a = 0.35
            markers.append(cube)

            # 添加ID文本标记
            text = Marker()
            text.header = cube.header
            text.ns = "mctrack_id"
            text.id = track_id * 10 + 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = pos[0]
            text.pose.position.y = pos[1]
            text.pose.position.z = pos[2] + height/2 + 0.5  # 在边界框上方显示ID
            text.scale.z = 1.0  # 文字大小
            text.color.r = r
            text.color.g = g
            text.color.b = b
            text.color.a = 1.0
            text.text = f"ID:{track_id}"
            markers.append(text)

        marker_array = MarkerArray(markers=markers)
        self.marker_pub.publish(marker_array)

    def publish_tracks(self, outputs: Dict[int, BBox], header: Header) -> None:
        stamp = header.stamp if header.stamp != rospy.Time() else rospy.Time.now()
        frame_id = header.frame_id or "camera_init"
        array_msg = TrackedObjectArray()
        array_msg.header = Header(stamp=stamp, frame_id=frame_id)

        for track_id, bbox in outputs.items():
            tracked = TrackedObject()
            tracked.header = Header(stamp=stamp, frame_id=frame_id)
            tracked.frame = bbox.frame_id
            tracked.track_id = track_id
            tracked.track_length = getattr(bbox, "track_length", 0)
            tracked.cls = bbox.category
            tracked.detection_score = float(bbox.det_score)
            tracked.yaw = float(bbox.global_yaw)
            tracked.dimensions = [float(dim) for dim in bbox.lwh]
            tracked.velocity = self._vec2(bbox.global_velocity)
            tracked.velocity_fusion = self._vec2(getattr(bbox, "global_velocity_fusion", bbox.global_velocity))
            tracked.velocity_diff = self._vec2(getattr(bbox, "global_velocity_diff", [0.0, 0.0]))
            tracked.velocity_curve = self._vec2(getattr(bbox, "global_velocity_curve", [0.0, 0.0]))
            tracked.acceleration = self._vec2(getattr(bbox, "global_acceleration", [0.0, 0.0]))

            pose = tracked.pose
            pose.position.x, pose.position.y, pose.position.z = [float(val) for val in bbox.global_xyz]
            quat = bbox.global_orientation
            pose.orientation.x = float(quat[1])
            pose.orientation.y = float(quat[2])
            pose.orientation.z = float(quat[3])
            pose.orientation.w = float(quat[0])

            lidar_pose = tracked.lidar_pose
            lidar_pose.position.x, lidar_pose.position.y, lidar_pose.position.z = [float(val) for val in bbox.lidar_xyz]
            lidar_quat = bbox.lidar_orientation
            lidar_pose.orientation.x = float(lidar_quat[1])
            lidar_pose.orientation.y = float(lidar_quat[2])
            lidar_pose.orientation.z = float(lidar_quat[3])
            lidar_pose.orientation.w = float(lidar_quat[0])
            tracked.lidar_yaw = float(bbox.lidar_yaw)

            array_msg.objects.append(tracked)

        self.track_pub.publish(array_msg)

    @staticmethod
    def _vec2(values: List[float]) -> List[float]:
        if values is None:
            return [0.0, 0.0]
        if len(values) >= 2:
            return [float(values[0]), float(values[1])]
        if len(values) == 1:
            return [float(values[0]), 0.0]
        return [0.0, 0.0]

    @staticmethod
    def hsv_to_rgb(h: float, s: float, v: float) -> tuple[float, float, float]:
        i = int(h * 6.0)
        f = h * 6.0 - i
        p = v * (1.0 - s)
        q = v * (1.0 - f * s)
        t = v * (1.0 - (1.0 - f) * s)
        i %= 6
        if i == 0:
            r, g, b = v, t, p
        elif i == 1:
            r, g, b = q, v, p
        elif i == 2:
            r, g, b = p, v, t
        elif i == 3:
            r, g, b = p, q, v
        elif i == 4:
            r, g, b = t, p, v
        else:
            r, g, b = v, p, q
        return r, g, b


def main() -> None:
    rospy.init_node("mctrack_online_node")
    node = MCTrackOnlineNode()
    rospy.spin()


if __name__ == "__main__":
    main()
