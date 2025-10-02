#!/usr/bin/env python3
"""Online MCTrack node consuming FAST-LIO pose and KITTI detections."""
from __future__ import annotations

import math
import sys
from collections import defaultdict
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np
import rospy
import yaml
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import Header
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
from kitti_tracklets_viz.msg import Detection3DArray  # noqa: E402


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


def get_global_yaw(rot_y: float, lidar2global: np.ndarray) -> float:
    cos_theta = lidar2global[0, 0]
    sin_theta = lidar2global[1, 0]
    cos_theta = max(-1.0, min(1.0, cos_theta))
    theta = math.acos(cos_theta)
    if sin_theta < 0:
        theta = 2 * math.pi - theta
    new_yaw = (math.pi - rot_y) + math.pi / 2.0
    return angle_wrap(new_yaw + theta)


class MCTrackOnlineNode:
    def __init__(self) -> None:
        self.seq = f"{int(rospy.get_param('~seq', 19)):04d}"
        dataset_param = rospy.get_param("~dataset_root", "tracking/training")
        dataset_root = Path(dataset_param)
        if not dataset_root.is_absolute():
            dataset_root = (REPO_ROOT / dataset_root).resolve()

        seq_root = dataset_root / "sequences" if (dataset_root / "sequences").is_dir() else dataset_root
        calib_dir = seq_root / "calib" if (seq_root / "calib").is_dir() else None
        if calib_dir and calib_dir.is_dir():
            calib_path = calib_dir / f"{self.seq}.txt"
        else:
            calib_path = seq_root / self.seq / "calib.txt"
        config_path = Path(rospy.get_param("~config", MCTrack_DIR / "config" / "kitti_fastlio.yaml"))
        self.pose_topic = rospy.get_param("~pose_topic", "/mctrack/lidar_pose")
        self.det_topic = rospy.get_param("~det_topic", "/kitti/detections")
        self.frame_rate = float(rospy.get_param("~frame_rate", 10.0))
        self.arrow_length = float(rospy.get_param("~arrow_length", 3.0))
        self.history_size = int(rospy.get_param("~history_size", 200))

        if not calib_path.exists():
            raise FileNotFoundError(f"Calibration file not found: {calib_path}")

        with config_path.open("r") as f:
            self.cfg = yaml.safe_load(f)

        self.tracker = Base3DTracker(self.cfg)
        calib_data = read_calib(calib_path)
        if "Tr_velo_cam" not in calib_data:
            raise ValueError("Calibration missing Tr_velo_cam")
        lidar2camera = np.vstack((calib_data["Tr_velo_cam"].reshape(3, 4), [0, 0, 0, 1]))
        camera2image = np.eye(4)
        camera2image[:3, :4] = calib_data["P2"].reshape(3, 4)
        self.camera2lidar = np.linalg.inv(lidar2camera)
        self.lidar2camera = lidar2camera
        self.camera2image = camera2image
        self.cameras_transform_matrix = build_camera_info(self.lidar2camera, self.camera2image)

        self.pose_matrix: Optional[np.ndarray] = None
        self.traj_history: Dict[int, List[np.ndarray]] = defaultdict(list)

        self.marker_pub = rospy.Publisher("/mctrack/markers", MarkerArray, queue_size=1)
        self.pose_sub = rospy.Subscriber(self.pose_topic, PoseStamped, self.pose_callback, queue_size=20)
        self.det_sub = rospy.Subscriber(self.det_topic, Detection3DArray, self.det_callback, queue_size=10)
        rospy.loginfo(
            "MCTrackOnlineNode initialized (seq=%s, calib=%s, config=%s)",
            self.seq,
            calib_path,
            config_path,
        )

    def pose_callback(self, msg: PoseStamped) -> None:
        self.pose_matrix = pose_to_matrix(msg)

    def det_callback(self, msg: Detection3DArray) -> None:
        if self.pose_matrix is None:
            rospy.logwarn_throttle(5.0, "No FAST-LIO pose yet; skipping detections")
            return
        frame_id = 0
        if msg.detections:
            frame_id = msg.detections[0].frame
        header = msg.header if msg.header.stamp != rospy.Time() else Header(stamp=rospy.Time.now())
        lidar2global = self.pose_matrix
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
            if cls not in cat_map:
                continue
            score_idx = cat_map[cls]
            score_thresh = online_scores[score_idx]
            if det.score < score_thresh:
                continue
            dims = np.array(det.dimensions, dtype=np.float64)
            lwh = [dims[2], dims[1], dims[0]]
            camera_xyz = np.array(det.location, dtype=np.float64)
            lidar_xyz = camera_to_lidar(camera_xyz, det.rotation_y, self.camera2lidar)
            lidar_xyz[2] += lwh[2] / 2.0
            global_xyz = project_global(lidar_xyz, lidar2global)
            global_yaw = get_global_yaw(det.rotation_y, lidar2global)
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
                "bbox_image": {
                    "camera_type": "CAM_FRONT",
                    "x1y1x2y2": list(map(float, det.bbox2d)),
                },
            }
            frame.bboxes.append(BBox(frame_id, bbox))

        if not frame.bboxes:
            return

        outputs = self.tracker.track_single_frame(frame)
        self.publish_markers(outputs, header)

    def publish_markers(self, outputs: Dict[int, BBox], header: Header) -> None:
        markers: List[Marker] = []
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.append(clear)
        stamp = header.stamp if header.stamp != rospy.Time() else rospy.Time.now()
        frame_id_base = "map"
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

            arrow = Marker()
            arrow.header = cube.header
            arrow.ns = "mctrack_heading"
            arrow.id = track_id * 10 + 1
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            start = Point(x=pos[0], y=pos[1], z=pos[2])
            end = Point(
                x=pos[0] + math.cos(yaw) * self.arrow_length,
                y=pos[1] + math.sin(yaw) * self.arrow_length,
                z=pos[2],
            )
            arrow.points = [start, end]
            arrow.scale.x = 0.2
            arrow.scale.y = 0.4
            arrow.scale.z = 0.6
            arrow.color.r = r
            arrow.color.g = g
            arrow.color.b = b
            arrow.color.a = 0.8
            markers.append(arrow)

            line = Marker()
            line.header = cube.header
            line.ns = "mctrack_traj"
            line.id = track_id * 10 + 2
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.scale.x = 0.1
            line.color.r = r
            line.color.g = g
            line.color.b = b
            line.color.a = 0.9
            line.points = [Point(x=p[0], y=p[1], z=p[2]) for p in history]
            markers.append(line)

        marker_array = MarkerArray(markers=markers)
        self.marker_pub.publish(marker_array)

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
