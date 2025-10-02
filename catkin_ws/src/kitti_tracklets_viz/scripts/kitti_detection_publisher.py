#!/usr/bin/env python3
"""Publish KITTI tracking detections as Detection3DArray messages."""
from __future__ import annotations

import sys
from pathlib import Path
from typing import Optional

import numpy as np
import rospy
from std_msgs.msg import Header

PACKAGE_DIR = Path(__file__).resolve()
REPO_ROOT = PACKAGE_DIR.parents[4]
SCRIPTS_DIR = REPO_ROOT / "Scripts"
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.append(str(SCRIPTS_DIR))

from kitti_tracking_loader import KITTITrackingLoader  # noqa: E402
from kitti_tracklets_viz.msg import Detection3D, Detection3DArray  # noqa: E402


class DetectionPublisher:
    def __init__(self) -> None:
        dataset_param = rospy.get_param("~dataset_root", "tracking")
        det_param = rospy.get_param("~detector_root", "")

        dataset_root = Path(dataset_param)
        if not dataset_root.is_absolute():
            dataset_root = (REPO_ROOT / dataset_root).resolve()
        self.dataset_root = dataset_root

        det_root_path: Optional[Path]
        if det_param:
            det_root_path = Path(det_param)
            if not det_root_path.is_absolute():
                det_root_path = (REPO_ROOT / det_root_path).resolve()
        else:
            det_root_path = None

        rospy.loginfo("dataset_root=%s", self.dataset_root)
        rospy.loginfo("det_root_path=%s", det_root_path)
        self.seq = f"{int(rospy.get_param('~seq', 0)):04d}"
        self.rate_hz = float(rospy.get_param("~rate", 10.0))
        self.loop = bool(rospy.get_param("~loop", False))

        self.loader = KITTITrackingLoader(self.dataset_root, det_root_path)
        self.publisher = rospy.Publisher("/kitti/detections", Detection3DArray, queue_size=2)
        self.frames = sorted(self.loader._load_detections(self.seq), key=lambda d: d.frame)
        self.unique_frames = sorted({det.frame for det in self.frames})
        if not self.unique_frames:
            rospy.logwarn("No detections found for sequence %s", self.seq)
        self.times = self.loader._load_times(self.seq)
        rospy.loginfo(
            "Loaded %d detections across %d frames for seq %s",
            len(self.frames),
            len(self.unique_frames),
            self.seq,
        )

    def publish(self) -> None:
        rate = rospy.Rate(self.rate_hz)
        index = 0
        while not rospy.is_shutdown():
            if index >= len(self.unique_frames):
                if self.loop:
                    index = 0
                else:
                    rospy.loginfo_once("All frames published. Stopping publisher.")
                    rospy.signal_shutdown("completed")
                    break
            frame_id = self.unique_frames[index]
            detections = [det for det in self.frames if det.frame == frame_id]
            msg = Detection3DArray()
            stamp = self._stamp_for_frame(frame_id)
            msg.header = Header(stamp=stamp, frame_id=f"seq_{self.seq}")
            for det in detections:
                det_msg = Detection3D()
                det_msg.header = Header(stamp=stamp, frame_id=f"seq_{self.seq}")
                det_msg.frame = det.frame
                det_msg.track_id = det.track_id
                det_msg.cls = det.cls
                det_msg.score = float(det.score)
                det_msg.bbox2d = np.asarray(det.bbox, dtype=np.float32).tolist()
                det_msg.dimensions = np.asarray(det.dimensions, dtype=np.float32).tolist()
                det_msg.location = np.asarray(det.location, dtype=np.float32).tolist()
                det_msg.rotation_y = float(det.rotation_y)
                msg.detections.append(det_msg)
            self.publisher.publish(msg)
            index += 1
            rate.sleep()

    def _stamp_for_frame(self, frame_id: int) -> rospy.Time:
        if self.times is not None and frame_id < len(self.times):
            return rospy.Time.from_sec(float(self.times[frame_id]))
        return rospy.Time.now()


def main() -> None:
    rospy.init_node("kitti_detection_publisher")
    pub = DetectionPublisher()
    pub.publish()


if __name__ == "__main__":
    main()
