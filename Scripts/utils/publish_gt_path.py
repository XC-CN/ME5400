#!/usr/bin/env python3
import argparse
from pathlib import Path as FilePath

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


def load_kitti_oxts(oxts_file):
    """Load KITTI OXTS rows as a dense NxM array."""
    path = FilePath(oxts_file)
    if not path.exists():
        return None

    if path.is_dir():
        files = sorted(path.glob("*.txt"))
        if not files:
            return None
        data = np.array([np.loadtxt(f) for f in files], dtype=np.float64)
    else:
        data = np.loadtxt(path, dtype=np.float64)

    if data.ndim == 1:
        data = data.reshape(1, -1)
    return data


def geodetic_to_local_enu(oxts_data):
    """Convert KITTI lat/lon/alt to a local ENU frame centered at the first pose."""
    lat = oxts_data[:, 0]
    lon = oxts_data[:, 1]
    alt = oxts_data[:, 2]

    earth_radius = 6378137.0
    lat0 = np.deg2rad(lat[0])
    lon0 = np.deg2rad(lon[0])
    alt0 = alt[0]

    east = earth_radius * np.cos(lat0) * (np.deg2rad(lon) - lon0)
    north = earth_radius * (np.deg2rad(lat) - lat0)
    up = alt - alt0
    return np.column_stack((east, north, up))


def rpy_to_matrix(roll, pitch, yaw):
    """KITTI OXTS uses roll/pitch/yaw for IMU pose in the global ENU frame."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    rx = np.array(
        [[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]],
        dtype=np.float64,
    )
    ry = np.array(
        [[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]],
        dtype=np.float64,
    )
    rz = np.array(
        [[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    return rz @ ry @ rx


def rotation_matrix_to_quaternion(rot):
    """Convert a 3x3 rotation matrix to xyzw quaternion."""
    trace = np.trace(rot)
    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        qw = 0.25 / s
        qx = (rot[2, 1] - rot[1, 2]) * s
        qy = (rot[0, 2] - rot[2, 0]) * s
        qz = (rot[1, 0] - rot[0, 1]) * s
    elif rot[0, 0] > rot[1, 1] and rot[0, 0] > rot[2, 2]:
        s = 2.0 * np.sqrt(1.0 + rot[0, 0] - rot[1, 1] - rot[2, 2])
        qw = (rot[2, 1] - rot[1, 2]) / s
        qx = 0.25 * s
        qy = (rot[0, 1] + rot[1, 0]) / s
        qz = (rot[0, 2] + rot[2, 0]) / s
    elif rot[1, 1] > rot[2, 2]:
        s = 2.0 * np.sqrt(1.0 + rot[1, 1] - rot[0, 0] - rot[2, 2])
        qw = (rot[0, 2] - rot[2, 0]) / s
        qx = (rot[0, 1] + rot[1, 0]) / s
        qy = 0.25 * s
        qz = (rot[1, 2] + rot[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + rot[2, 2] - rot[0, 0] - rot[1, 1])
        qw = (rot[1, 0] - rot[0, 1]) / s
        qx = (rot[0, 2] + rot[2, 0]) / s
        qy = (rot[1, 2] + rot[2, 1]) / s
        qz = 0.25 * s
    return np.array([qx, qy, qz, qw], dtype=np.float64)


def build_camera_init_poses(oxts_data):
    """
    Convert OXTS ENU poses into FAST-LIO's camera_init frame.

    FAST-LIO initializes camera_init as the first IMU/body frame rather than
    a geographic ENU frame. We therefore rotate all GT poses by the inverse of
    the first OXTS attitude before publishing to RViz.
    """
    positions_enu = geodetic_to_local_enu(oxts_data)
    rotations_world_from_imu = [
        rpy_to_matrix(roll, pitch, yaw)
        for roll, pitch, yaw in oxts_data[:, 3:6]
    ]

    first_rotation = rotations_world_from_imu[0]
    rotation_camera_init_from_world = first_rotation.T

    relative_positions = positions_enu - positions_enu[0]
    positions_camera_init = (
        rotation_camera_init_from_world @ relative_positions.T
    ).T
    rotations_camera_init_from_imu = [
        rotation_camera_init_from_world @ rot
        for rot in rotations_world_from_imu
    ]
    quaternions = [
        rotation_matrix_to_quaternion(rot) for rot in rotations_camera_init_from_imu
    ]
    return positions_camera_init, quaternions

def main():
    parser = argparse.ArgumentParser(description="Publish Ground Truth Trajectory to RViz")
    parser.add_argument("gt_file", help="Path to the Ground Truth file/directory (KITTI Oxts)")
    parser.add_argument(
        "--frame-id",
        default="camera_init",
        help="Target frame for the published GT path (default: camera_init)",
    )
    args = parser.parse_args()

    rospy.init_node("gt_path_publisher", anonymous=True)
    
    # Create publisher with latch=True so late joiners like RViz get the path
    pub = rospy.Publisher("/gt_path", Path, queue_size=1, latch=True)
    
    rospy.loginfo(f"Loading Ground Truth from: {args.gt_file}")
    oxts_data = load_kitti_oxts(args.gt_file)

    if oxts_data is None or len(oxts_data) == 0:
        rospy.logerr("Failed to load Ground Truth trajectory.")
        return

    gt_xyz, gt_quat = build_camera_init_poses(oxts_data)
    initial_yaw_deg = np.rad2deg(oxts_data[0, 5])
    rospy.loginfo(
        "GT path aligned into %s using first OXTS yaw %.2f deg",
        args.frame_id,
        initial_yaw_deg,
    )

    # Create Path message
    path_msg = Path()
    path_msg.header.stamp = rospy.Time.now()
    path_msg.header.frame_id = args.frame_id

    # Populate Path with points
    for i in range(len(gt_xyz)):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = args.frame_id

        pose.pose.position.x = gt_xyz[i, 0]
        pose.pose.position.y = gt_xyz[i, 1]
        pose.pose.position.z = gt_xyz[i, 2]

        pose.pose.orientation.x = gt_quat[i][0]
        pose.pose.orientation.y = gt_quat[i][1]
        pose.pose.orientation.z = gt_quat[i][2]
        pose.pose.orientation.w = gt_quat[i][3]

        path_msg.poses.append(pose)

    rospy.loginfo(f"Publishing {len(path_msg.poses)} trace points to /gt_path")
    pub.publish(path_msg)

    # Keep the node alive so the latched topic remains available
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
