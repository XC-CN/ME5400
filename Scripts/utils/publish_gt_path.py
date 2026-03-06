#!/usr/bin/env python3
import rospy
import numpy as np
import argparse
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from pathlib import Path as FilePath

def load_kitti_oxts(oxts_file):
    """Load KITTI Oxts data and convert to local Cartesian coordinates."""
    path = FilePath(oxts_file)
    if not path.exists():
        return None
        
    if path.is_dir():
        files = sorted(path.glob("*.txt"))
        data = []
        for f in files:
            data.append(np.loadtxt(f))
        data = np.array(data)
    else:
        data = np.loadtxt(path)
    
    if data.ndim == 1:
        data = data.reshape(1, -1)

    # Extract lat, lon, alt (first 3 columns)
    lat = data[:, 0]
    lon = data[:, 1]
    alt = data[:, 2]
    
    # Convert to local Cartesian (Flat Earth approximation around start)
    R_earth = 6378137.0
    lat0 = np.deg2rad(lat[0])
    lon0 = np.deg2rad(lon[0])
    alt0 = alt[0]
    
    x = R_earth * np.cos(lat0) * np.deg2rad(lon - data[0, 1])
    y = R_earth * np.deg2rad(lat - data[0, 0])
    z = alt - alt0
    
    return np.column_stack((x, y, z))

def main():
    parser = argparse.ArgumentParser(description="Publish Ground Truth Trajectory to RViz")
    parser.add_argument("gt_file", help="Path to the Ground Truth file/directory (KITTI Oxts)")
    args = parser.parse_args()

    rospy.init_node("gt_path_publisher", anonymous=True)
    
    # Create publisher with latch=True so late joiners like RViz get the path
    pub = rospy.Publisher("/gt_path", Path, queue_size=1, latch=True)
    
    rospy.loginfo(f"Loading Ground Truth from: {args.gt_file}")
    gt_xyz = load_kitti_oxts(args.gt_file)
    
    if gt_xyz is None or len(gt_xyz) == 0:
        rospy.logerr("Failed to load Ground Truth trajectory.")
        return

    # Create Path message
    path_msg = Path()
    path_msg.header.stamp = rospy.Time.now()
    path_msg.header.frame_id = "camera_init"  # Lidar odometry base frame in FAST-LIO

    # Populate Path with points
    for i in range(len(gt_xyz)):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "camera_init"
        
        pose.pose.position.x = gt_xyz[i, 0]
        pose.pose.position.y = gt_xyz[i, 1]
        pose.pose.position.z = gt_xyz[i, 2]
        
        # We only have XYZ from oxts lat/lon conversion in this script, leaving orientation default (Identity)
        pose.pose.orientation.w = 1.0
        
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
