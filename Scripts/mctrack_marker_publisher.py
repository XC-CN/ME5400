#!/usr/bin/env python3
"""在 ROS 中发布 MCTrack 的 3D 目标框到 MarkerArray。"""
import argparse
from pathlib import Path
from typing import Dict, List

import numpy as np
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import quaternion_from_matrix

DATA_ROOT = Path(__file__).resolve().parents[1] / "MCTrack" / "data" / "kitti"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Publish MCTrack results to RViz")
    parser.add_argument("--result", required=True, help="结果文件路径，例如 results/.../data/0000.txt")
    parser.add_argument("--calib", required=True, help="对应的标定文件路径，例如 data/kitti/datasets/training/calib/0000.txt")
    parser.add_argument("--frame", default="velo_link", help="发布的坐标系")
    parser.add_argument("--rate", type=float, default=10.0, help="播放频率")
    parser.add_argument("--loop", action="store_true", help="循环播放")
    return parser.parse_args()


def load_calib(calib_path: Path) -> Dict[str, np.ndarray]:
    data: Dict[str, np.ndarray] = {}
    with calib_path.open("r") as f:
        for raw in f:
            line = raw.strip()
            if not line:
                continue
            tokens = line.split()
            key = tokens[0].rstrip(":")
            values = np.array([float(x) for x in tokens[1:]], dtype=np.float64)
            data[key] = values
    return data


def read_result(result_path: Path) -> Dict[int, List[List[float]]]:
    frames: Dict[int, List[List[float]]] = {}
    with result_path.open("r") as f:
        for line in f:
            parts = line.strip().split()
            if not parts:
                continue
            frame = int(parts[0])
            track_id = int(parts[1])
            obj_type = parts[2]
            h, w, l = map(float, parts[10:13])
            x, y, z = map(float, parts[13:16])
            yaw = float(parts[16])
            score = float(parts[17]) if len(parts) > 17 else 1.0
            frames.setdefault(frame, []).append([
                track_id,
                obj_type,
                h,
                w,
                l,
                x,
                y,
                z,
                yaw,
                score,
            ])
    return frames


def camera_to_velo(calib: Dict[str, np.ndarray]):
    Tr = calib["Tr_velo_cam"].reshape(3, 4)
    T_cam_velo = np.vstack((Tr, np.array([0.0, 0.0, 0.0, 1.0])))
    T_velo_cam = np.linalg.inv(T_cam_velo)
    R_cam_velo = T_cam_velo[:3, :3]
    R_velo_cam = R_cam_velo.T
    t_velo_cam = T_velo_cam[:3, 3]
    return T_velo_cam, R_velo_cam, R_cam_velo


def make_marker_template(frame_id: str) -> Marker:
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 0.5
    marker.lifetime = rospy.Duration(1.0 / 30.0)
    return marker


def main():
    args = parse_args()
    result_path = Path(args.result)
    calib_path = Path(args.calib)

    frames = read_result(result_path)
    if not frames:
        rospy.logerr("未在结果文件中找到数据")
        return
    calib = load_calib(calib_path)
    T_velo_cam, R_velo_cam, R_cam_velo = camera_to_velo(calib)

    rospy.init_node("mctrack_marker_publisher")
    pub = rospy.Publisher("/mctrack/markers", MarkerArray, queue_size=1)
    rate = rospy.Rate(args.rate)

    ordered_frames = sorted(frames.items())
    while not rospy.is_shutdown():
        for frame_id, objs in ordered_frames:
            stamp = rospy.Time.now()
            clear_msg = Marker()
            clear_msg.action = Marker.DELETEALL
            pub.publish(MarkerArray(markers=[clear_msg]))

            markers: List[Marker] = []
            for det_idx, det in enumerate(objs):
                track_id, obj_type, h, w, l, x, y, z, yaw, score = det
                center_cam = np.array([x, y - h / 2.0, z, 1.0])
                center_velo = T_velo_cam @ center_cam

                cos_y, sin_y = np.cos(yaw), np.sin(yaw)
                R_cam = np.array(
                    [[cos_y, 0.0, sin_y], [0.0, 1.0, 0.0], [-sin_y, 0.0, cos_y]],
                    dtype=np.float64,
                )
                R_velo = R_velo_cam @ R_cam @ R_cam_velo
                T_velo = np.eye(4)
                T_velo[:3, :3] = R_velo
                T_velo[:3, 3] = center_velo[:3]
                quat = quaternion_from_matrix(T_velo)

                marker = make_marker_template(args.frame)
                marker.header.stamp = stamp
                marker.ns = obj_type
                marker.id = track_id
                marker.pose.position.x = center_velo[0]
                marker.pose.position.y = center_velo[1]
                marker.pose.position.z = center_velo[2]
                marker.pose.orientation.x = quat[0]
                marker.pose.orientation.y = quat[1]
                marker.pose.orientation.z = quat[2]
                marker.pose.orientation.w = quat[3]
                marker.scale.x = l
                marker.scale.y = w
                marker.scale.z = h
                marker.color.a = min(max(score, 0.3), 1.0)
                markers.append(marker)

            pub.publish(MarkerArray(markers=markers))
            rate.sleep()
            if rospy.is_shutdown():
                break
        if not args.loop:
            break


if __name__ == "__main__":
    main()
