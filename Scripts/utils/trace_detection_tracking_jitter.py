#!/usr/bin/env python3
"""Capture detection/tracking timing to diagnose visual jitter."""
from __future__ import annotations

import argparse
import csv
import math
import os
import time
from collections import deque
from dataclasses import dataclass
from typing import Deque, List, Optional, Tuple

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

from ME5400.msg import Detection3DArray


def stamp_to_sec(stamp: rospy.Time) -> float:
    return float(stamp.to_sec()) if stamp != rospy.Time() else float("nan")


def now_sec() -> float:
    return float(rospy.Time.now().to_sec())


@dataclass
class PoseSample:
    stamp: float
    x: float
    y: float
    z: float


def pose_from_pose_stamped(msg: PoseStamped) -> PoseSample:
    return PoseSample(
        stamp=stamp_to_sec(msg.header.stamp),
        x=float(msg.pose.position.x),
        y=float(msg.pose.position.y),
        z=float(msg.pose.position.z),
    )


def pose_from_odom(msg: Odometry) -> PoseSample:
    pose = msg.pose.pose.position
    return PoseSample(
        stamp=stamp_to_sec(msg.header.stamp),
        x=float(pose.x),
        y=float(pose.y),
        z=float(pose.z),
    )


def vec_dist(a: PoseSample, b: PoseSample) -> float:
    return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)


def first_add_marker(msg: MarkerArray) -> Optional[Marker]:
    for marker in msg.markers:
        if marker.action == Marker.ADD:
            return marker
    return None


class JitterTracer:
    def __init__(self, output_prefix: str, buffer_size: int) -> None:
        self.output_prefix = output_prefix
        self.pose_buffer: Deque[PoseSample] = deque(maxlen=buffer_size)
        self.odom_buffer: Deque[PoseSample] = deque(maxlen=buffer_size)
        self.latest_pose: Optional[PoseSample] = None
        self.latest_odom: Optional[PoseSample] = None

        self.det_rows: List[List[object]] = []
        self.pp_marker_rows: List[List[object]] = []
        self.track_marker_rows: List[List[object]] = []

    def pose_cb(self, msg: PoseStamped) -> None:
        sample = pose_from_pose_stamped(msg)
        self.latest_pose = sample
        self.pose_buffer.append(sample)

    def odom_cb(self, msg: Odometry) -> None:
        sample = pose_from_odom(msg)
        self.latest_odom = sample
        self.odom_buffer.append(sample)

    def det_cb(self, msg: Detection3DArray) -> None:
        det_stamp = stamp_to_sec(msg.header.stamp)
        sim_now = now_sec()
        latest_pose = self.latest_pose
        nearest_pose = self._nearest_pose(self.pose_buffer, det_stamp)
        latest_delta = (
            latest_pose.stamp - det_stamp if latest_pose is not None and math.isfinite(det_stamp) else float("nan")
        )
        nearest_delta = (
            nearest_pose.stamp - det_stamp if nearest_pose is not None and math.isfinite(det_stamp) else float("nan")
        )
        latest_nearest_gap = (
            vec_dist(latest_pose, nearest_pose)
            if latest_pose is not None and nearest_pose is not None
            else float("nan")
        )
        first = msg.detections[0] if msg.detections else None
        self.det_rows.append(
            [
                sim_now,
                det_stamp,
                sim_now - det_stamp if math.isfinite(det_stamp) else float("nan"),
                len(msg.detections),
                latest_pose.stamp if latest_pose is not None else float("nan"),
                latest_delta,
                nearest_pose.stamp if nearest_pose is not None else float("nan"),
                nearest_delta,
                latest_nearest_gap,
                float(first.location[0]) if first else float("nan"),
                float(first.location[1]) if first else float("nan"),
                float(first.location[2]) if first else float("nan"),
            ]
        )

    def pp_marker_cb(self, msg: MarkerArray) -> None:
        marker = first_add_marker(msg)
        if marker is None:
            return
        stamp = stamp_to_sec(marker.header.stamp)
        self.pp_marker_rows.append(
            [
                now_sec(),
                stamp,
                now_sec() - stamp if math.isfinite(stamp) else float("nan"),
                marker.header.frame_id,
                int(marker.frame_locked),
                float(marker.pose.position.x),
                float(marker.pose.position.y),
                float(marker.pose.position.z),
            ]
        )

    def track_marker_cb(self, msg: MarkerArray) -> None:
        marker = first_add_marker(msg)
        if marker is None:
            return
        stamp = stamp_to_sec(marker.header.stamp)
        self.track_marker_rows.append(
            [
                now_sec(),
                stamp,
                now_sec() - stamp if math.isfinite(stamp) else float("nan"),
                marker.header.frame_id,
                int(marker.frame_locked),
                marker.ns,
                marker.id,
                float(marker.pose.position.x),
                float(marker.pose.position.y),
                float(marker.pose.position.z),
            ]
        )

    @staticmethod
    def _nearest_pose(buffer: Deque[PoseSample], stamp: float) -> Optional[PoseSample]:
        if not buffer or not math.isfinite(stamp):
            return None
        return min(buffer, key=lambda sample: abs(sample.stamp - stamp))

    def write_outputs(self) -> None:
        os.makedirs(os.path.dirname(self.output_prefix), exist_ok=True)
        self._write_csv(
            f"{self.output_prefix}_detections.csv",
            [
                "arrival_sim_sec",
                "det_stamp_sec",
                "publish_lag_sec",
                "num_detections",
                "latest_pose_stamp_sec",
                "latest_pose_minus_det_sec",
                "nearest_pose_stamp_sec",
                "nearest_pose_minus_det_sec",
                "latest_vs_nearest_pose_gap_m",
                "first_det_x",
                "first_det_y",
                "first_det_z",
            ],
            self.det_rows,
        )
        self._write_csv(
            f"{self.output_prefix}_pp_markers.csv",
            [
                "arrival_sim_sec",
                "marker_stamp_sec",
                "publish_lag_sec",
                "frame_id",
                "frame_locked",
                "x",
                "y",
                "z",
            ],
            self.pp_marker_rows,
        )
        self._write_csv(
            f"{self.output_prefix}_track_markers.csv",
            [
                "arrival_sim_sec",
                "marker_stamp_sec",
                "publish_lag_sec",
                "frame_id",
                "frame_locked",
                "ns",
                "id",
                "x",
                "y",
                "z",
            ],
            self.track_marker_rows,
        )

    @staticmethod
    def _write_csv(path: str, header: List[str], rows: List[List[object]]) -> None:
        with open(path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(header)
            writer.writerows(rows)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=15.0, help="Capture duration in seconds")
    parser.add_argument(
        "--output-prefix",
        default="/tmp/me5400_jitter/jitter_trace",
        help="Output prefix without suffix",
    )
    parser.add_argument("--buffer-size", type=int, default=256, help="Pose buffer size")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rospy.init_node("trace_detection_tracking_jitter", anonymous=True)
    tracer = JitterTracer(output_prefix=args.output_prefix, buffer_size=args.buffer_size)

    rospy.Subscriber("/mctrack/lidar_pose", PoseStamped, tracer.pose_cb, queue_size=100)
    rospy.Subscriber("/Odometry", Odometry, tracer.odom_cb, queue_size=100)
    rospy.Subscriber("/detection/lidar_detections", Detection3DArray, tracer.det_cb, queue_size=100)
    rospy.Subscriber("/detection/bboxes_3d", MarkerArray, tracer.pp_marker_cb, queue_size=100)
    rospy.Subscriber("/mctrack/markers", MarkerArray, tracer.track_marker_cb, queue_size=100)

    wall_deadline = time.monotonic() + args.duration
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        if time.monotonic() >= wall_deadline:
            break
        rate.sleep()

    tracer.write_outputs()
    print(f"Wrote {args.output_prefix}_detections.csv")
    print(f"Wrote {args.output_prefix}_pp_markers.csv")
    print(f"Wrote {args.output_prefix}_track_markers.csv")


if __name__ == "__main__":
    main()
