#!/usr/bin/env python3
"""Bridge FAST-LIO odometry to PoseStamped and TF for MCTrack."""
from __future__ import annotations

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros
from tf.transformations import quaternion_from_matrix, quaternion_matrix


def odom_to_pose(msg: Odometry) -> PoseStamped:
    pose_msg = PoseStamped()
    pose_msg.header = msg.header
    pose_msg.pose = msg.pose.pose
    return pose_msg


def pose_to_matrix(pose_msg: PoseStamped) -> np.ndarray:
    quat = pose_msg.pose.orientation
    T = quaternion_matrix([quat.x, quat.y, quat.z, quat.w])
    T[0, 3] = pose_msg.pose.position.x
    T[1, 3] = pose_msg.pose.position.y
    T[2, 3] = pose_msg.pose.position.z
    return T


def matrix_to_pose(T: np.ndarray, header) -> PoseStamped:
    pose_msg = PoseStamped()
    pose_msg.header = header
    quat = quaternion_from_matrix(T)
    pose_msg.pose.position.x = float(T[0, 3])
    pose_msg.pose.position.y = float(T[1, 3])
    pose_msg.pose.position.z = float(T[2, 3])
    pose_msg.pose.orientation.x = float(quat[0])
    pose_msg.pose.orientation.y = float(quat[1])
    pose_msg.pose.orientation.z = float(quat[2])
    pose_msg.pose.orientation.w = float(quat[3])
    return pose_msg


class FastLioPoseBridge:
    def __init__(self) -> None:
        self.odom_topic = rospy.get_param("~odom_topic", "/Odometry")
        self.pose_topic = rospy.get_param("~pose_topic", "/mctrack/lidar_pose")
        self.frame_id = rospy.get_param("~frame_id", "camera_init")
        self.child_frame_id = rospy.get_param("~child_frame_id", "velodyne")
        self.publish_tf = bool(rospy.get_param("~publish_tf", True))

        extrinsic_t = rospy.get_param(
            "~extrinsic_t",
            rospy.get_param("/mapping/extrinsic_T", rospy.get_param("mapping/extrinsic_T", [0.0, 0.0, 0.0])),
        )
        extrinsic_r = rospy.get_param(
            "~extrinsic_r",
            rospy.get_param(
                "/mapping/extrinsic_R",
                rospy.get_param("mapping/extrinsic_R", [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]),
            ),
        )
        self.body_to_lidar = np.eye(4, dtype=np.float64)
        self.body_to_lidar[:3, :3] = np.asarray(extrinsic_r, dtype=np.float64).reshape(3, 3)
        self.body_to_lidar[:3, 3] = np.asarray(extrinsic_t, dtype=np.float64)

        self.pose_pub = rospy.Publisher(self.pose_topic, PoseStamped, queue_size=10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        self.sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb, queue_size=50)
        rospy.loginfo(
            "fastlio_pose_bridge listening on %s, publishing %s (child=%s, publish_tf=%s)",
            self.odom_topic,
            self.pose_topic,
            self.child_frame_id,
            self.publish_tf,
        )

    def odom_cb(self, msg: Odometry) -> None:
        body_pose_msg = odom_to_pose(msg)
        if body_pose_msg.header.frame_id == "":
            body_pose_msg.header.frame_id = self.frame_id
        if body_pose_msg.header.frame_id != self.frame_id:
            rospy.logwarn_once(
                "Pose frame %s differs from configured %s",
                body_pose_msg.header.frame_id,
                self.frame_id,
            )
        world_from_body = pose_to_matrix(body_pose_msg)
        world_from_lidar = world_from_body @ self.body_to_lidar
        pose_msg = matrix_to_pose(world_from_lidar, body_pose_msg.header)
        self.pose_pub.publish(pose_msg)

        if not self.publish_tf:
            return

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
