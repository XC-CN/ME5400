#!/usr/bin/env python3
"""Convert KITTI Tracking velodyne + OXTS (+ detections) to ROS bag."""
import argparse
import math
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np
import rosbag
import rospy
from geometry_msgs.msg import Quaternion, TransformStamped
from sensor_msgs.msg import Imu, PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage

THIS_DIR = Path(__file__).resolve().parent
REPO_ROOT = THIS_DIR.parent
if str(THIS_DIR) not in sys.path:
    sys.path.append(str(THIS_DIR))

from kitti_tracking_loader import KITTITrackingLoader  # noqa: E402
try:
    from kitti_tracklets_viz.msg import Detection3D, Detection3DArray  # noqa: E402
except ImportError:  # pragma: no cover - optional dependency
    Detection3D = Detection3DArray = None


def str2bool(value):
    if isinstance(value, bool):
        return value
    value = value.lower()
    if value in {"1", "true", "t", "yes", "y"}:
        return True
    if value in {"0", "false", "f", "no", "n"}:
        return False
    raise argparse.ArgumentTypeError(f"Boolean value expected, got '{value}'")


def load_velodyne_file(path: Path) -> np.ndarray:
    data = np.fromfile(str(path), dtype=np.float32)
    if data.size % 4 != 0:
        raise ValueError(f"Unexpected velodyne file size: {path}")
    return data.reshape(-1, 4)


VELODYNE_NUM_SCANS = 64
VELODYNE_MIN_VERT_ANGLE = -24.9  # degrees
VELODYNE_MAX_VERT_ANGLE = 2.0    # degrees


def estimate_ring_time(points: np.ndarray, scan_rate: float) -> tuple[np.ndarray, np.ndarray]:
    """Estimate ring index and per-point time offsets for KITTI Velodyne scans."""
    if points.size == 0:
        return np.array([], dtype=np.uint16), np.array([], dtype=np.float32)

    scan_rate = scan_rate if scan_rate > 0 else 10.0
    scan_period = 1.0 / scan_rate

    xy_norm = np.linalg.norm(points[:, :2], axis=1)
    safe_xy = np.where(xy_norm > 1e-6, xy_norm, 1e-6)
    vert_angle = np.degrees(np.arctan2(points[:, 2], safe_xy))
    vert_angle = np.clip(vert_angle, VELODYNE_MIN_VERT_ANGLE, VELODYNE_MAX_VERT_ANGLE)

    relative = (vert_angle - VELODYNE_MIN_VERT_ANGLE) / (VELODYNE_MAX_VERT_ANGLE - VELODYNE_MIN_VERT_ANGLE)
    ring = np.floor(relative * VELODYNE_NUM_SCANS).astype(np.int32)
    ring = np.clip(ring, 0, VELODYNE_NUM_SCANS - 1)

    ring_int = ring.astype(np.int32)
    counts = np.bincount(ring_int, minlength=VELODYNE_NUM_SCANS)
    progress = np.zeros(VELODYNE_NUM_SCANS, dtype=np.int32)
    times = np.zeros(points.shape[0], dtype=np.float64)

    for idx, r in enumerate(ring_int):
        denom = counts[r] - 1
        if denom <= 0:
            times[idx] = 0.0
        else:
            times[idx] = (progress[r] / denom) * scan_period
        progress[r] += 1

    ring = ring.astype(np.uint16)
    time_us = (times * 1e6).astype(np.float32)
    return ring, time_us


def create_pointcloud2(points: np.ndarray, timestamp: float, frame_id: str, scan_rate: float) -> PointCloud2:
    header = Header()
    header.stamp = rospy.Time.from_sec(timestamp)
    header.frame_id = frame_id

    ring, time_us = estimate_ring_time(points, scan_rate)

    dtype = np.dtype([
        ('x', np.float32),
        ('y', np.float32),
        ('z', np.float32),
        ('pad', np.float32),
        ('intensity', np.float32),
        ('time', np.float32),
        ('ring', np.uint16),
        ('pad2', np.uint16),
        ('pad3', np.uint32),
    ], align=True)

    structured = np.zeros(points.shape[0], dtype=dtype)
    structured['x'] = points[:, 0].astype(np.float32, copy=False)
    structured['y'] = points[:, 1].astype(np.float32, copy=False)
    structured['z'] = points[:, 2].astype(np.float32, copy=False)
    structured['intensity'] = points[:, 3].astype(np.float32, copy=False)
    structured['time'] = time_us
    structured['ring'] = ring

    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        PointField('intensity', 16, PointField.FLOAT32, 1),
        PointField('time', 20, PointField.FLOAT32, 1),
        PointField('ring', 24, PointField.UINT16, 1),
    ]

    cloud = PointCloud2()
    cloud.header = header
    cloud.height = 1
    cloud.width = points.shape[0]
    cloud.fields = fields
    cloud.is_bigendian = False
    cloud.point_step = structured.dtype.itemsize
    cloud.row_step = cloud.point_step * cloud.width
    cloud.is_dense = True
    cloud.data = structured.tobytes()
    return cloud


def oxts_line_to_dict(values):
    keys = [
        "lat", "lon", "alt", "roll", "pitch", "yaw",
        "vn", "ve", "vf", "vl", "vu",
        "ax", "ay", "az", "af", "al", "au",
        "wx", "wy", "wz", "wf", "wl", "wu",
        "pos_accuracy", "vel_accuracy",
        "navstat", "numsats", "posmode", "velmode", "orimode",
    ]
    return {k: values[i] for i, k in enumerate(keys)}


def euler_to_quaternion(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    q = Quaternion()
    q.w = cy * cp * cr + sy * sp * sr
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr
    return q


def create_imu(oxts: dict, timestamp: float, frame_id: str) -> Imu:
    imu = Imu()
    imu.header.stamp = rospy.Time.from_sec(timestamp)
    imu.header.frame_id = frame_id
    imu.orientation = euler_to_quaternion(oxts["roll"], oxts["pitch"], oxts["yaw"])
    imu.angular_velocity.x = oxts["wx"]
    imu.angular_velocity.y = oxts["wy"]
    imu.angular_velocity.z = oxts["wz"]
    imu.linear_acceleration.x = oxts["ax"]
    imu.linear_acceleration.y = oxts["ay"]
    imu.linear_acceleration.z = oxts["az"]
    return imu


def load_calibration(dataset_root: Path, seq: str) -> dict[str, np.ndarray]:
    seq_root = dataset_root / "sequences" if (dataset_root / "sequences").is_dir() else dataset_root
    calib_dir = seq_root / "calib" if (seq_root / "calib").is_dir() else None
    if calib_dir and calib_dir.is_dir():
        calib_file = calib_dir / f"{seq}.txt"
    else:
        calib_file = seq_root / seq / "calib.txt"
    if not calib_file.exists():
        raise FileNotFoundError(f"Calibration file not found: {calib_file}")

    data: dict[str, np.ndarray] = {}
    with calib_file.open("r") as f:
        for line in f:
            parts = line.strip().split()
            if not parts:
                continue
            key = parts[0].rstrip(":")
            values = [float(v) for v in parts[1:]]
            data[key] = np.asarray(values, dtype=np.float64)
    return data


def rotation_matrix_to_quaternion(R: np.ndarray) -> tuple[float, float, float, float]:
    m00, m01, m02 = R[0]
    m10, m11, m12 = R[1]
    m20, m21, m22 = R[2]
    trace = m00 + m11 + m22
    if trace > 0.0:
        s = 0.5 / math.sqrt(trace + 1.0)
        qw = 0.25 / s
        qx = (m21 - m12) * s
        qy = (m02 - m20) * s
        qz = (m10 - m01) * s
    elif m00 > m11 and m00 > m22:
        s = 2.0 * math.sqrt(1.0 + m00 - m11 - m22)
        qw = (m21 - m12) / s
        qx = 0.25 * s
        qy = (m01 + m10) / s
        qz = (m02 + m20) / s
    elif m11 > m22:
        s = 2.0 * math.sqrt(1.0 + m11 - m00 - m22)
        qw = (m02 - m20) / s
        qx = (m01 + m10) / s
        qy = 0.25 * s
        qz = (m12 + m21) / s
    else:
        s = 2.0 * math.sqrt(1.0 + m22 - m00 - m11)
        qw = (m10 - m01) / s
        qx = (m02 + m20) / s
        qy = (m12 + m21) / s
        qz = 0.25 * s
    return qx, qy, qz, qw


def default_detector_root(dataset_root: Path) -> Path:
    """Infer detector root based on dataset split."""
    base = dataset_root.parent / "det_tracking_lsvm"
    split = dataset_root.name
    candidate = base / split
    if (candidate / "det_02").is_dir():
        return candidate / "det_02"
    return candidate


def load_times(dataset_root: Path, seq: str):
    seq_root = dataset_root / "sequences" if (dataset_root / "sequences").is_dir() else dataset_root
    time_file = seq_root / seq / "times.txt"
    if not time_file.exists():
        return None
    values = []
    for line in time_file.read_text().strip().splitlines():
        try:
            values.append(float(line))
        except ValueError:
            continue
    return values if values else None


def detection_array_message(dets, stamp: rospy.Time, seq: str) -> Detection3DArray:
    msg = Detection3DArray()
    frame_id = f"seq_{seq}"
    msg.header = Header(stamp=stamp, frame_id=frame_id)
    for det in dets:
        det_msg = Detection3D()
        det_msg.header = Header(stamp=stamp, frame_id=frame_id)
        det_msg.frame = det.frame
        det_msg.track_id = det.track_id
        det_msg.cls = det.cls
        det_msg.score = float(det.score)
        det_msg.bbox2d = np.asarray(det.bbox, dtype=np.float32).tolist()
        det_msg.dimensions = np.asarray(det.dimensions, dtype=np.float32).tolist()
        det_msg.location = np.asarray(det.location, dtype=np.float32).tolist()
        det_msg.rotation_y = float(det.rotation_y)
        msg.detections.append(det_msg)
    return msg


def main():
    parser = argparse.ArgumentParser(description="KITTI Tracking to rosbag")
    parser.add_argument("--dataset_root", type=Path, default=Path("tracking/training"))
    parser.add_argument("--seq", type=str, default="0019", help="sequence id, e.g., 0000")
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="output bag path (defaults to tracking/rosbags/seq_<seq>_<with|no>det.bag)",
    )
    parser.add_argument("--frame_id", type=str, default="velodyne")
    parser.add_argument("--imu_frame", type=str, default="imu")
    parser.add_argument("--rate", type=float, default=10.0)
    parser.add_argument(
        "--detector_root",
        type=Path,
        default=None,
        help="Detection root containing <seq>.txt (defaults to det_tracking_lsvm/<split>/det_02)",
    )
    parser.add_argument(
        "--include_detections",
        type=str2bool,
        default=False,
        help="Include 3D detections in the bag (True/False)",
    )
    args = parser.parse_args()

    seq = f"{int(args.seq):04d}" if args.seq.isdigit() else args.seq
    dataset_root = args.dataset_root
    if not dataset_root.is_absolute():
        dataset_root = (REPO_ROOT / dataset_root).resolve()
    output_path = args.output
    if output_path is None:
        suffix = "with_det" if args.include_detections else "nodet"
        output_path = Path(f"tracking/rosbags/seq_{seq}_{suffix}.bag")
    if not output_path.is_absolute():
        output_path = (REPO_ROOT / output_path).resolve()

    velodyne_dir = dataset_root / "velodyne" / seq
    oxts_file = dataset_root / "oxts" / f"{seq}.txt"

    if not velodyne_dir.is_dir():
        raise FileNotFoundError(f"Velodyne directory not found: {velodyne_dir}")
    if not oxts_file.is_file():
        raise FileNotFoundError(f"OXTS file not found: {oxts_file}")

    frame_files = sorted(velodyne_dir.glob("*.bin"))
    if not frame_files:
        raise RuntimeError(f"No velodyne frames in {velodyne_dir}")

    with oxts_file.open("r") as f:
        oxts_lines = [list(map(float, line.strip().split())) for line in f if line.strip()]

    times = load_times(dataset_root, seq)
    timestamps = [
        (times[i] if times and i < len(times) else i / args.rate)
        for i in range(max(len(frame_files), len(oxts_lines)))
    ]

    if timestamps:
        min_time = min(timestamps)
        if min_time <= 0.0:
            shift = abs(min_time) + 1e-3
            timestamps = [t + shift for t in timestamps]

    detections_by_frame = {}
    if args.include_detections:
        if Detection3D is None or Detection3DArray is None:
            raise ImportError(
                "kitti_tracklets_viz ROS messages not found; install package or run with --include_detections False"
            )
        det_root = args.detector_root or default_detector_root(dataset_root)
        if det_root and not det_root.is_absolute():
            det_root = (REPO_ROOT / det_root).resolve()
        loader = KITTITrackingLoader(dataset_root, det_root)
        detections_by_frame = defaultdict(list)
        for det in loader._load_detections(seq):
            detections_by_frame[det.frame].append(det)

    output_path.parent.mkdir(parents=True, exist_ok=True)

    calib = load_calibration(dataset_root, seq)
    static_tf_msg = None
    if "Tr_imu_velo" in calib:
        mat = calib["Tr_imu_velo"].reshape(3, 4)
        rot = mat[:, :3]
        trans = mat[:, 3]
        qx, qy, qz, qw = rotation_matrix_to_quaternion(rot)
        static_tf = TransformStamped()
        stamp_value = timestamps[0] if timestamps else 0.0
        static_tf.header.stamp = rospy.Time.from_sec(stamp_value)
        static_tf.header.frame_id = args.imu_frame
        static_tf.child_frame_id = args.frame_id
        static_tf.transform.translation.x = float(trans[0])
        static_tf.transform.translation.y = float(trans[1])
        static_tf.transform.translation.z = float(trans[2])
        static_tf.transform.rotation.x = float(qx)
        static_tf.transform.rotation.y = float(qy)
        static_tf.transform.rotation.z = float(qz)
        static_tf.transform.rotation.w = float(qw)
        static_tf_msg = TFMessage(transforms=[static_tf])
    else:
        raise KeyError("Tr_imu_velo not found in calibration data; cannot create static TF.")

    with rosbag.Bag(str(output_path), "w") as bag:
        if static_tf_msg is not None:
            bag.write("/tf_static", static_tf_msg, static_tf_msg.transforms[0].header.stamp)
        for idx, frame_path in enumerate(frame_files):
            timestamp = timestamps[idx]
            points = load_velodyne_file(frame_path)
            pc_msg = create_pointcloud2(points, timestamp, args.frame_id, args.rate)
            bag.write("/kitti/velo/pointcloud", pc_msg, pc_msg.header.stamp)

            if idx < len(oxts_lines):
                oxts_dict = oxts_line_to_dict(oxts_lines[idx])
                imu_msg = create_imu(oxts_dict, timestamp, args.imu_frame)
                bag.write("/kitti/oxts/imu", imu_msg, imu_msg.header.stamp)

            if detections_by_frame and detections_by_frame.get(idx):
                det_stamp = pc_msg.header.stamp
                det_msg = detection_array_message(detections_by_frame[idx], det_stamp, seq)
                bag.write("/kitti/detections", det_msg, det_msg.header.stamp)

            if idx % 100 == 0:
                print(f"已处理帧 {idx}/{len(frame_files)}")

    print(f"已保存 rosbag 文件到 {output_path}")


if __name__ == "__main__":
    main()
