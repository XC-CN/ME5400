#!/usr/bin/env python3
"""Bridge FAST-LIO odometry to PoseStamped and TF for MCTrack."""
from __future__ import annotations

import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros


def odom_to_pose(msg: Odometry) -> PoseStamped:
    pose_msg = PoseStamped()
    pose_msg.header = msg.header
    pose_msg.pose = msg.pose.pose
    return pose_msg


class FastLioPoseBridge:
    def __init__(self) -> None:
        self.odom_topic = rospy.get_param("~odom_topic", "/Odometry")
        self.pose_topic = rospy.get_param("~pose_topic", "/mctrack/lidar_pose")
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.child_frame_id = rospy.get_param("~child_frame_id", "velodyne")

        self.pose_pub = rospy.Publisher(self.pose_topic, PoseStamped, queue_size=10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        self.sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb, queue_size=50)
        rospy.loginfo(
            "fastlio_pose_bridge listening on %s, publishing %s",
            self.odom_topic,
            self.pose_topic,
        )

    def odom_cb(self, msg: Odometry) -> None:
        pose_msg = odom_to_pose(msg)
        if pose_msg.header.frame_id == "":
            pose_msg.header.frame_id = self.frame_id
        if pose_msg.header.frame_id != self.frame_id:
            rospy.logwarn_once(
                "Pose frame %s differs from configured %s",
                pose_msg.header.frame_id,
                self.frame_id,
            )
        self.pose_pub.publish(pose_msg)

        tf_msg = TransformStamped()
        tf_msg.header = pose_msg.header
        tf_msg.child_frame_id = self.child_frame_id
        tf_msg.transform.translation.x = pose_msg.pose.position.x
        tf_msg.transform.translation.y = pose_msg.pose.position.y
        tf_msg.transform.translation.z = pose_msg.pose.position.z
        tf_msg.transform.rotation = pose_msg.pose.orientation
        self.tf_broadcaster.sendTransform(tf_msg)


def main() -> None:
    rospy.init_node("fastlio_pose_bridge")
    FastLioPoseBridge()
    rospy.spin()


if __name__ == "__main__":
    main()
