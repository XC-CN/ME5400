#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Record nav_msgs/Odometry to a TUM trajectory text file.

Output format per line:
  timestamp x y z qx qy qz qw
"""

import argparse
from pathlib import Path

import rospy
from nav_msgs.msg import Odometry


class OdomToTumRecorder:
    def __init__(self, output_path: Path):
        self.output_path = output_path
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        if self.output_path.exists():
            rospy.logwarn("[odom_to_tum] overwrite existing file: %s", self.output_path)
            self.output_path.unlink()

        self.file = self.output_path.open("w")
        self.count = 0
        self.last_stamp = None

        self.odom_topic = rospy.get_param("~odom_topic", "/joint_backend/odom")
        self.min_dt = float(rospy.get_param("~min_dt", 0.0))

        self.sub = rospy.Subscriber(self.odom_topic, Odometry, self.cb, queue_size=200)
        rospy.loginfo(
            "[odom_to_tum] recording topic=%s -> %s (min_dt=%.4f)",
            self.odom_topic,
            str(self.output_path),
            self.min_dt,
        )

    def cb(self, msg: Odometry):
        stamp = msg.header.stamp.to_sec()
        if stamp <= 0.0:
            return

        if self.last_stamp is not None:
            if stamp < self.last_stamp:
                # Ignore out-of-order messages (may happen on time reset/loop).
                return
            if (stamp - self.last_stamp) < self.min_dt:
                return

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.file.write(
            f"{stamp:.6f} {p.x:.9f} {p.y:.9f} {p.z:.9f} "
            f"{q.x:.9f} {q.y:.9f} {q.z:.9f} {q.w:.9f}\n"
        )
        self.count += 1
        self.last_stamp = stamp

        if self.count % 100 == 0:
            rospy.loginfo("[odom_to_tum] recorded %d poses", self.count)

    def shutdown(self):
        if not self.file.closed:
            self.file.close()
        rospy.loginfo(
            "[odom_to_tum] saved %d poses to %s",
            self.count,
            str(self.output_path),
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output",
        default="Results/0020_results/trajectory_joint.txt",
        help="Output TUM trajectory path",
    )
    args, unknown = parser.parse_known_args()

    output_path = Path(args.output).resolve()

    rospy.init_node("odom_to_tum_recorder", argv=unknown, anonymous=False)
    recorder = OdomToTumRecorder(output_path)
    rospy.on_shutdown(recorder.shutdown)
    rospy.spin()


if __name__ == "__main__":
    main()
