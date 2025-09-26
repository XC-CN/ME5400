#!/usr/bin/python3
# 固定使用系统 /usr/bin/python3（与ROS Noetic打包一致），避免激活 Conda 3.13 环境导致缺少 rospkg。
# -*- coding: utf-8 -*-
"""Publish KITTI tracklet 3D bounding boxes to RViz as MarkerArray.

Assumptions:
 - Playing KITTI rosbag that provides /clock and point clouds in /kitti/velo/pointcloud
 - We use /clock to sync simulation time, or fallback to wall time.
 - tracklet_labels.xml path is provided via param ~tracklet_file

Topics:
 - Publishes: /kitti/tracklets_markers (visualization_msgs/MarkerArray)

Usage (example):
 rosrun kitti_tracklets_viz tracklets_publisher.py _tracklet_file:=/home/xc/Projects/ME5400/KITTI_Data/2011_09_26/2011_09_26_drive_0019_sync/tracklet_labels.xml

"""
from __future__ import annotations
import rospy
import xml.etree.ElementTree as ET
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import math

class TrackletObject:
    def __init__(self, obj_type, h, w, l, first_frame, poses):
        self.type = obj_type
        self.h = float(h)
        self.w = float(w)
        self.l = float(l)
        self.first_frame = int(first_frame)
        # each pose: dict(x,y,z, rx, ry, rz)
        self.poses = poses

    def pose_at_frame(self, frame_idx):
        rel = frame_idx - self.first_frame
        if rel < 0 or rel >= len(self.poses):
            return None
        return self.poses[rel]

CLASS_COLOR = {
    'Car': (0.1, 0.9, 0.1),
    'Van': (0.9, 0.6, 0.1),
    'Cyclist': (0.1, 0.4, 0.9),
    'Pedestrian': (0.9, 0.1, 0.1)
}

class TrackletsPublisher:
    def __init__(self):
        # ---- 参数读取 ----
        self.tracklet_file = rospy.get_param('~tracklet_file', '')
        if not self.tracklet_file:
            rospy.logerr('Parameter ~tracklet_file is required.')
            raise SystemExit(1)
        self.frame_rate = float(rospy.get_param('~frame_rate', 10.0))  # KITTI velodyne ~10Hz
        self.base_frame = rospy.get_param('~frame_id', 'velo_link')
        self.topic = rospy.get_param('~topic', '/kitti/tracklets_markers')
        self.start_frame = int(rospy.get_param('~start_frame', 0))
        self.max_frame = int(rospy.get_param('~max_frame', -1))
        self.offset_time = float(rospy.get_param('~time_offset', 0.0))
        self.include_types = rospy.get_param('~include_types', '')  # 逗号分隔, 为空表示全部
        self.enable_text = bool(int(rospy.get_param('~enable_text', 1)))
        # 轨迹参数
        self.enable_traj = bool(int(rospy.get_param('~enable_traj', 1)))
        self.traj_history_len = int(rospy.get_param('~traj_history_len', 300))  # 每目标最多保存点数
        self.traj_line_width = float(rospy.get_param('~traj_line_width', 0.08))

        if self.include_types:
            self.include_types = {t.strip() for t in self.include_types.split(',') if t.strip()}
        else:
            self.include_types = None

        rospy.loginfo('Loading tracklets from %s', self.tracklet_file)
        self.tracklets = self._load_tracklets(self.tracklet_file)
        rospy.loginfo('Frame rate=%.2f start_frame=%d offset_time=%.2f frame_id=%s',
                      self.frame_rate, self.start_frame, self.offset_time, self.base_frame)
        if self.include_types:
            rospy.loginfo('Filtering object types: %s', ','.join(sorted(self.include_types)))
        if not rospy.get_param('/use_sim_time', False):
            rospy.logwarn_once('当前未启用 /use_sim_time。如果使用 rosbag --clock，建议: rosparam set use_sim_time true')

        # 发布器
        self.publisher = rospy.Publisher(self.topic, MarkerArray, queue_size=1)

        # 时间与状态
        self.start_time_wall = rospy.Time.now()
        self.last_debug_print = 0.0
        self.current_frame = self.start_frame

        # 轨迹历史: {object_index: [ (x,y,z), ... ]}
        self.traj_history = {}

        # 定时器
        self.timer = rospy.Timer(rospy.Duration(1.0/self.frame_rate), self._on_timer)

    def _load_tracklets(self, path):
        tree = ET.parse(path)
        root = tree.getroot()
        tracklets_elem = root.find('tracklets')
        objects = []
        # structure: tracklets/item ...
        for item in tracklets_elem.findall('item'):
            obj_type = item.findtext('objectType')
            h = item.findtext('h')
            w = item.findtext('w')
            l = item.findtext('l')
            first_frame = item.findtext('first_frame')
            poses_elem = item.find('poses')
            poses = []
            if poses_elem is not None:
                for pose_item in poses_elem.findall('item'):
                    # pose fields: tx, ty, tz, rx, ry, rz
                    pose = {}
                    for tag in ['tx','ty','tz','rx','ry','rz']:
                        val = pose_item.findtext(tag)
                        if val is None:
                            continue
                        pose[tag] = float(val)
                    if len(pose) == 6:
                        poses.append(pose)
            obj = TrackletObject(obj_type, h, w, l, first_frame, poses)
            objects.append(obj)
        rospy.loginfo('Loaded %d tracklets', len(objects))
        return objects

    def _on_timer(self, _):
        # 使用 sim time (若启用) 计算当前帧；否则退回 wall 时间
        now_ros = rospy.Time.now()
        elapsed = (now_ros - self.start_time_wall).to_sec() + self.offset_time
        self.current_frame = int(self.start_frame + elapsed * self.frame_rate)
        if self.max_frame > 0 and self.current_frame > self.max_frame:
            rospy.loginfo_throttle(5.0, 'Reached max_frame %d (current %d)', self.max_frame, self.current_frame)
            return

        markers = MarkerArray()
        stamp = now_ros
        mid = 0
        box_count = 0
        traj_count = 0
        for obj_idx, obj in enumerate(self.tracklets):
            if self.include_types and obj.type not in self.include_types:
                continue
            pose = obj.pose_at_frame(self.current_frame)
            if pose is None:
                continue
            # BOX
            box_marker = self._make_box_marker(mid, obj, pose, stamp)
            markers.markers.append(box_marker)
            mid += 1
            box_count += 1
            # TEXT
            if self.enable_text:
                text_marker = self._make_text_marker(mid, obj, pose, stamp, box_marker)
                markers.markers.append(text_marker)
                mid += 1
            # TRAJECTORY
            if self.enable_traj:
                hlist = self.traj_history.setdefault(obj_idx, [])
                hlist.append((pose['tx'], pose['ty'], pose['tz']))
                # 限制长度
                if len(hlist) > self.traj_history_len:
                    del hlist[0:len(hlist)-self.traj_history_len]
                traj_marker = self._make_traj_marker(mid, obj, hlist, stamp)
                markers.markers.append(traj_marker)
                mid += 1
                traj_count += 1

        self.publisher.publish(markers)

        # 周期性调试输出（每 ~1 秒）
        if stamp.to_sec() - self.last_debug_print > 1.0:
            self.last_debug_print = stamp.to_sec()
            rospy.loginfo('[frame %d] boxes=%d traj=%d published markers=%d',
                          self.current_frame, box_count, traj_count, len(markers.markers))

    def _make_box_marker(self, marker_id, obj, pose, stamp):
        m = Marker()
        m.header.frame_id = self.base_frame
        m.header.stamp = stamp
        m.ns = 'kitti_tracklets'
        m.id = marker_id
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.scale.x = obj.l
        m.scale.y = obj.w
        m.scale.z = obj.h
        # KITTI coordinate to ROS (assuming LiDAR frame):
        # KITTI: x-forward, y-left, z-up. ROS typical: x-forward, y-left, z-up (Velodyne). So direct.
        m.pose.position.x = pose['tx']
        m.pose.position.y = pose['ty']
        m.pose.position.z = pose['tz']
        # Orientation: only rz (yaw) is meaningful normally; rx, ry small. We'll build quaternion.
        q = euler_to_quaternion(pose['rx'], pose['ry'], pose['rz'])
        m.pose.orientation.x = q[0]
        m.pose.orientation.y = q[1]
        m.pose.orientation.z = q[2]
        m.pose.orientation.w = q[3]
        color = CLASS_COLOR.get(obj.type, (0.8,0.8,0.8))
        m.color.r, m.color.g, m.color.b = color
        m.color.a = 0.35
        m.lifetime = rospy.Duration(1.0/self.frame_rate * 1.5)
        return m

    def _make_text_marker(self, marker_id, obj, pose, stamp, ref_marker):
        m = Marker()
        m.header.frame_id = ref_marker.header.frame_id
        m.header.stamp = stamp
        m.ns = 'kitti_tracklets_text'
        m.id = marker_id
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.scale.z = 0.8
        m.pose.position.x = ref_marker.pose.position.x
        m.pose.position.y = ref_marker.pose.position.y
        m.pose.position.z = ref_marker.pose.position.z + ref_marker.scale.z * 0.6
        m.text = obj.type
        m.color.r, m.color.g, m.color.b = CLASS_COLOR.get(obj.type, (1,1,1))
        m.color.a = 1.0
        m.lifetime = rospy.Duration(1.0/self.frame_rate * 1.5)
        return m

    def _make_traj_marker(self, marker_id, obj, history_points, stamp):
        m = Marker()
        m.header.frame_id = self.base_frame
        m.header.stamp = stamp
        m.ns = 'kitti_tracklets_traj'
        m.id = marker_id
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = self.traj_line_width  # 线宽
        color = CLASS_COLOR.get(obj.type, (1,1,1))
        m.color.r, m.color.g, m.color.b = color
        m.color.a = 0.9
        m.pose.orientation.w = 1.0
        m.lifetime = rospy.Duration(1.0/self.frame_rate * 1.5)
        for (x, y, z) in history_points:
            p = Point(x=x, y=y, z=z)
            m.points.append(p)
        return m


def euler_to_quaternion(roll, pitch, yaw):
    # standard conversion
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return (x,y,z,w)


def main():
    rospy.init_node('kitti_tracklets_publisher')
    TrackletsPublisher()
    rospy.loginfo('KITTI tracklets publisher started.')
    rospy.spin()

if __name__ == '__main__':
    main()
