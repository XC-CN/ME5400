#!/usr/bin/env python3
"""Record FAST-LIO odometry synchronized with Velodyne scans into KITTI pose text files."""
import argparse
from pathlib import Path

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from tf.transformations import quaternion_matrix
import message_filters


def odom_to_matrix(msg: Odometry) -> np.ndarray:
    quat = msg.pose.pose.orientation
    trans = msg.pose.pose.position
    q = [quat.x, quat.y, quat.z, quat.w]
    T = quaternion_matrix(q)
    T[:3, 3] = [trans.x, trans.y, trans.z]
    return T


def format_matrix(T: np.ndarray) -> str:
    mat = T[:3, :]
    return " ".join(f"{v:.12e}" for v in mat.reshape(-1))


class FastLioPoseRecorder:
    def __init__(self, output_root: Path, seq: str):
        self.output_root = output_root
        self.output_root.mkdir(parents=True, exist_ok=True)
        self.seq = seq
        self.output_path = self.output_root / f"{seq}.txt"
        if self.output_path.exists():
            rospy.logwarn("Overwriting existing pose file: %s", self.output_path)
            self.output_path.unlink()
        self.file = self.output_path.open("w")
        self.frame_id = 0

        odom_topic = rospy.get_param("~odom_topic", "/aft_mapped_to_init")
        point_topic = rospy.get_param("~point_topic", "/detection/pointcloud")
        queue = rospy.get_param("~queue_size", 50)
        slop = rospy.get_param("~time_slop", 0.05)

        self.pc_sub = message_filters.Subscriber(point_topic, PointCloud2)
        self.odom_sub = message_filters.Subscriber(odom_topic, Odometry)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.pc_sub, self.odom_sub], queue_size=queue, slop=slop
        )
        self.sync.registerCallback(self.callback)
        rospy.loginfo(
            "Recording FAST-LIO poses to %s (queue=%d, slop=%.3f)",
            self.output_path,
            queue,
            slop,
        )

    def callback(self, pc_msg: PointCloud2, odom_msg: Odometry):
        T = odom_to_matrix(odom_msg)
        line = format_matrix(T)
        self.file.write(line + "\n")
        self.frame_id += 1
        if self.frame_id % 50 == 0:
            rospy.loginfo("Recorded %d frames", self.frame_id)

    def shutdown(self):
        if not self.file.closed:
            self.file.close()
            rospy.loginfo(
                "Pose recorder saved %d frames to %s", self.frame_id, self.output_path
            )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", default="MCTrack/data/kitti/datasets/training/pose_fastlio")
    parser.add_argument("--seq", default="0000")
    args, unknown = parser.parse_known_args()

    output_root = Path(args.output)
    rospy.init_node("fastlio_pose_recorder", argv=unknown, anonymous=False)
    recorder = FastLioPoseRecorder(output_root, args.seq)
    rospy.on_shutdown(recorder.shutdown)
    rospy.spin()


if __name__ == "__main__":
    main()
