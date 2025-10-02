#!/usr/bin/env python3
"""Convert KITTI Tracking velodyne + OXTS to ROS bag."""
import argparse
import math
from pathlib import Path

import numpy as np
import rosbag
import rospy
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import PointCloud2, PointField, Imu
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header


def load_velodyne_file(path: Path) -> np.ndarray:
    data = np.fromfile(str(path), dtype=np.float32)
    if data.size % 4 != 0:
        raise ValueError(f"Unexpected velodyne file size: {path}")
    return data.reshape(-1, 4)


def create_pointcloud2(points: np.ndarray, timestamp: float, frame_id: str) -> PointCloud2:
    header = Header()
    header.stamp = rospy.Time.from_sec(timestamp)
    header.frame_id = frame_id
    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        PointField('intensity', 12, PointField.FLOAT32, 1),
    ]
    return pc2.create_cloud(header, fields, points)


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


def main():
    parser = argparse.ArgumentParser(description="KITTI Tracking to rosbag")
    parser.add_argument("--dataset_root", type=Path, default=Path("tracking/training"))
    parser.add_argument("--seq", type=str, required=True, help="sequence id, e.g., 0000")
    parser.add_argument("--output", type=Path, required=True, help="output bag path")
    parser.add_argument("--frame_id", type=str, default="velodyne")
    parser.add_argument("--imu_frame", type=str, default="imu")
    parser.add_argument("--rate", type=float, default=10.0)
    args = parser.parse_args()

    seq = f"{int(args.seq):04d}" if args.seq.isdigit() else args.seq
    dataset_root = args.dataset_root
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

    timestamps = [i / args.rate for i in range(max(len(frame_files), len(oxts_lines)))]

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with rosbag.Bag(str(args.output), "w") as bag:
        for idx, frame_path in enumerate(frame_files):
            timestamp = timestamps[idx]
            points = load_velodyne_file(frame_path)
            pc_msg = create_pointcloud2(points, timestamp, args.frame_id)
            bag.write("/kitti/velo/pointcloud", pc_msg, pc_msg.header.stamp)

            if idx < len(oxts_lines):
                oxts_dict = oxts_line_to_dict(oxts_lines[idx])
                imu_msg = create_imu(oxts_dict, timestamp, args.imu_frame)
                bag.write("/kitti/oxts/imu", imu_msg, imu_msg.header.stamp)

            if idx % 100 == 0:
                print(f"Processed frame {idx}/{len(frame_files)}")

    print(f"Saved rosbag to {args.output}")


if __name__ == "__main__":
    main()
