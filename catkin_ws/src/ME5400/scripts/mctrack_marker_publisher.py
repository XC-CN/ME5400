#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Publish MCTrack KITTI tracking results as MarkerArray for RViz visualization.

This node reads a KITTI tracking result file (as produced by MCTrack) and
publishes 3D bounding boxes/labels/trajectories in the Velodyne coordinate frame.

Parameters
----------
~result_file : str
    Path to the MCTrack tracking result txt for a single sequence (e.g. 0000.txt).
~calib_file : str
    Path to the KITTI calibration file (calib/XXXX.txt) for the same sequence.
~frame_rate : float (default 10.0)
    Playback rate (frames per second).
~frame_id : str (default 'velo_link')
    Marker fixed frame id.
~start_frame : int (default 0)
    Start frame index.
~max_frame : int (default -1)
    Maximum frame index (inclusive). -1 means use last frame in results.
~loop : bool (default True)
    Loop when reaching the end.
~score_threshold : float (default 0.0)
    Minimum detection score to visualize.
~include_types : str (default '')
    Comma separated list of object types to include (e.g. 'Car,Pedestrian').
~enable_text : bool (default True)
    Publish text markers showing track id.
~enable_traj : bool (default True)
    Publish trajectory lines for each track.
~traj_history_len : int (default 200)
    Maximum number of points stored per trajectory.
~traj_line_width : float (default 0.1)
    Line width for trajectory markers.

"""
from __future__ import annotations

import math
from pathlib import Path
from typing import Dict, List

import numpy as np
import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

_CLASS_COLOR = {
    'car': (0.1, 0.9, 0.1),
    'pedestrian': (0.9, 0.1, 0.1),
    'cyclist': (0.1, 0.4, 0.9),
    'van': (0.9, 0.6, 0.1),
    'person': (0.9, 0.1, 0.1),
}


def _parse_calib(calib_path: Path) -> Dict[str, np.ndarray]:
    calib: Dict[str, np.ndarray] = {}
    with calib_path.open('r') as f:
        for raw in f:
            line = raw.strip()
            if not line:
                continue
            if ':' in line:
                key, values = line.split(':', 1)
            else:
                parts = line.split()
                if not parts:
                    continue
                key, values = parts[0], ' '.join(parts[1:])
            try:
                calib[key.strip()] = np.array([float(x) for x in values.split()])
            except ValueError:
                rospy.logwarn('无法解析标定行: %s', line)
    return calib


def _rotation_y(yaw: float) -> np.ndarray:
    cos_t = math.cos(yaw)
    sin_t = math.sin(yaw)
    return np.array([[cos_t, 0.0, sin_t],
                     [0.0, 1.0, 0.0],
                     [-sin_t, 0.0, cos_t]], dtype=np.float32)


def _matrix_to_quaternion(rot: np.ndarray) -> np.ndarray:
    """Convert 3x3 rotation matrix to quaternion [x, y, z, w]."""
    q = np.empty(4, dtype=np.float32)
    trace = np.trace(rot)
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        q[3] = 0.25 * s
        q[0] = (rot[2, 1] - rot[1, 2]) / s
        q[1] = (rot[0, 2] - rot[2, 0]) / s
        q[2] = (rot[1, 0] - rot[0, 1]) / s
    else:
        diag = np.diag(rot)
        idx = int(np.argmax(diag))
        if idx == 0:
            s = math.sqrt(1.0 + rot[0, 0] - rot[1, 1] - rot[2, 2]) * 2.0
            q[3] = (rot[2, 1] - rot[1, 2]) / s
            q[0] = 0.25 * s
            q[1] = (rot[0, 1] + rot[1, 0]) / s
            q[2] = (rot[0, 2] + rot[2, 0]) / s
        elif idx == 1:
            s = math.sqrt(1.0 + rot[1, 1] - rot[0, 0] - rot[2, 2]) * 2.0
            q[3] = (rot[0, 2] - rot[2, 0]) / s
            q[0] = (rot[0, 1] + rot[1, 0]) / s
            q[1] = 0.25 * s
            q[2] = (rot[1, 2] + rot[2, 1]) / s
        else:
            s = math.sqrt(1.0 + rot[2, 2] - rot[0, 0] - rot[1, 1]) * 2.0
            q[3] = (rot[1, 0] - rot[0, 1]) / s
            q[0] = (rot[0, 2] + rot[2, 0]) / s
            q[1] = (rot[1, 2] + rot[2, 1]) / s
            q[2] = 0.25 * s
    norm = math.sqrt(float(np.dot(q, q)))
    if norm > 0.0:
        q /= norm
    else:
        q[:] = (0.0, 0.0, 0.0, 1.0)
    return q


class _Detection:
    __slots__ = (
        'frame', 'track_id', 'obj_type', 'dims', 'position', 'quaternion', 'score'
    )

    def __init__(self, frame: int, track_id: int, obj_type: str,
                 dims: np.ndarray, position: np.ndarray,
                 quaternion: np.ndarray, score: float) -> None:
        self.frame = frame
        self.track_id = track_id
        self.obj_type = obj_type
        self.dims = dims
        self.position = position
        self.quaternion = quaternion
        self.score = score


class MCTrackMarkerPublisher:
    def __init__(self) -> None:
        result_file = rospy.get_param('~result_file', '')
        calib_file = rospy.get_param('~calib_file', '')
        if not result_file or not calib_file:
            rospy.logerr('~result_file 与 ~calib_file 参数均不能为空')
            raise SystemExit(1)

        self._frame_rate = float(rospy.get_param('~frame_rate', 10.0))
        self._frame_id = rospy.get_param('~frame_id', 'velo_link')
        self._start_frame = int(rospy.get_param('~start_frame', 0))
        self._max_frame = int(rospy.get_param('~max_frame', -1))
        self._loop = bool(int(rospy.get_param('~loop', 1)))
        self._score_th = float(rospy.get_param('~score_threshold', 0.0))
        include_types = rospy.get_param('~include_types', '')
        self._include_types = {t.strip().lower() for t in include_types.split(',') if t.strip()} or None
        self._enable_text = bool(int(rospy.get_param('~enable_text', 1)))
        self._enable_traj = bool(int(rospy.get_param('~enable_traj', 1)))
        self._traj_history_len = int(rospy.get_param('~traj_history_len', 200))
        self._traj_line_width = float(rospy.get_param('~traj_line_width', 0.1))

        rospy.loginfo('Loading calibration: %s', calib_file)
        calib = _parse_calib(Path(calib_file))
        if 'Tr_velo_cam' not in calib:
            rospy.logerr('未在标定文件中找到 Tr_velo_cam，无法完成坐标转换')
            raise SystemExit(1)

        T_cam_velo = np.eye(4, dtype=np.float32)
        Tr = calib['Tr_velo_cam'].reshape(3, 4)
        T_cam_velo[:3, :4] = Tr
        T_velo_cam = np.linalg.inv(T_cam_velo)
        R_cv = Tr[:3, :3]
        self._R_vc = R_cv.T  # camera -> velo rotation
        self._T_velo_cam = T_velo_cam

        rospy.loginfo('Loading tracking results: %s', result_file)
        self._frames = self._load_results(Path(result_file))
        if not self._frames:
            rospy.logwarn('未在结果文件中解析出任何目标')
        self._frame_ids = sorted(self._frames.keys())
        if not self._frame_ids:
            self._frame_ids = [0]
        self._current_index = 0
        # reset to start
        self._seek_to_frame(self._start_frame)

        self._traj_history: Dict[int, List[np.ndarray]] = {}

        self._publisher = rospy.Publisher('~markers', MarkerArray, queue_size=1)
        self._timer = rospy.Timer(rospy.Duration(1.0 / max(self._frame_rate, 1e-3)), self._on_timer)
        self._clear_markers_once()

    def _clear_markers_once(self) -> None:
        msg = MarkerArray()
        m = Marker()
        m.header.stamp = rospy.Time.now()
        m.header.frame_id = self._frame_id
        m.action = Marker.DELETEALL
        msg.markers.append(m)
        self._publisher.publish(msg)

    def _seek_to_frame(self, target_frame: int) -> None:
        if not self._frame_ids:
            return
        idx = 0
        for i, frame in enumerate(self._frame_ids):
            if frame >= target_frame:
                idx = i
                break
        else:
            idx = len(self._frame_ids) - 1
        self._current_index = idx

    def _load_results(self, result_path: Path) -> Dict[int, List[_Detection]]:
        frames: Dict[int, List[_Detection]] = {}
        with result_path.open('r') as f:
            for raw in f:
                line = raw.strip()
                if not line:
                    continue
                parts = line.split()
                if len(parts) < 17:
                    continue
                frame = int(float(parts[0]))
                track_id = int(float(parts[1]))
                obj_type = parts[2].lower()
                if self._include_types and obj_type not in self._include_types:
                    continue
                score = float(parts[17]) if len(parts) > 17 else 1.0
                if score < self._score_th:
                    continue
                h = float(parts[10])
                w = float(parts[11])
                l = float(parts[12])
                x = float(parts[13])
                y = float(parts[14])
                z = float(parts[15])
                rot_y = float(parts[16])

                pos_cam = np.array([x, y, z, 1.0], dtype=np.float32)
                pos_velo_h = self._T_velo_cam @ pos_cam
                position = pos_velo_h[:3]

                R_cam = _rotation_y(rot_y)
                R_velo = self._R_vc @ R_cam @ self._R_vc.T
                quat = _matrix_to_quaternion(R_velo)

                det = _Detection(
                    frame=frame,
                    track_id=track_id,
                    obj_type=obj_type,
                    dims=np.array([l, w, h], dtype=np.float32),
                    position=position,
                    quaternion=quat,
                    score=score,
                )
                frames.setdefault(frame, []).append(det)
        return frames

    def _on_timer(self, event) -> None:
        if not self._frame_ids:
            return
        frame_id = self._frame_ids[self._current_index]
        detections = self._frames.get(frame_id, [])
        stamp = rospy.Time.now()

        markers = MarkerArray()
        marker_idx = 0
        for det in detections:
            color = _CLASS_COLOR.get(det.obj_type, (0.3, 0.7, 0.9))
            # bounding box marker
            m = Marker()
            m.header.stamp = stamp
            m.header.frame_id = self._frame_id
            m.ns = f"track_{det.track_id}"
            m.id = marker_idx
            marker_idx += 1
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = float(det.position[0])
            m.pose.position.y = float(det.position[1])
            m.pose.position.z = float(det.position[2])
            m.pose.orientation.x = float(det.quaternion[0])
            m.pose.orientation.y = float(det.quaternion[1])
            m.pose.orientation.z = float(det.quaternion[2])
            m.pose.orientation.w = float(det.quaternion[3])
            m.scale.x = float(det.dims[0])
            m.scale.y = float(det.dims[1])
            m.scale.z = float(det.dims[2])
            m.color.r, m.color.g, m.color.b = color
            m.color.a = 0.35
            m.lifetime = rospy.Duration(1.5 / max(self._frame_rate, 1e-3))
            markers.markers.append(m)

            if self._enable_text:
                t = Marker()
                t.header = m.header
                t.ns = m.ns + '_text'
                t.id = marker_idx
                marker_idx += 1
                t.type = Marker.TEXT_VIEW_FACING
                t.action = Marker.ADD
                t.pose.position.x = m.pose.position.x
                t.pose.position.y = m.pose.position.y
                t.pose.position.z = m.pose.position.z + det.dims[2] * 0.5 + 0.3
                t.scale.z = 0.8
                t.color.r, t.color.g, t.color.b = color
                t.color.a = 0.9
                t.text = f"ID {det.track_id}"
                t.lifetime = m.lifetime
                markers.markers.append(t)

            if self._enable_traj:
                hist = self._traj_history.setdefault(det.track_id, [])
                hist.append(det.position.copy())
                if len(hist) > self._traj_history_len:
                    del hist[0:len(hist) - self._traj_history_len]
                traj = Marker()
                traj.header = m.header
                traj.ns = m.ns + '_traj'
                traj.id = marker_idx
                marker_idx += 1
                traj.type = Marker.LINE_STRIP
                traj.action = Marker.ADD
                traj.scale.x = self._traj_line_width
                traj.color.r, traj.color.g, traj.color.b = color
                traj.color.a = 0.8
                traj.lifetime = m.lifetime
                traj.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in hist]
                markers.markers.append(traj)

        self._publisher.publish(markers)

        if self._max_frame >= 0 and frame_id >= self._max_frame:
            if self._loop:
                self._current_index = 0
            else:
                rospy.loginfo_once('已达到最大帧，停止播放')
                self._timer.shutdown()
            return

        self._current_index += 1
        if self._current_index >= len(self._frame_ids):
            if self._loop:
                self._current_index = 0
            else:
                self._timer.shutdown()


def main() -> None:
    rospy.init_node('mctrack_marker_publisher')
    try:
        MCTrackMarkerPublisher()
        rospy.loginfo('MCTrack marker publisher started')
        rospy.spin()
    except SystemExit:
        pass
    except Exception as exc:  # pylint: disable=broad-except
        rospy.logerr('MCTrack marker publisher failed: %s', exc)


if __name__ == '__main__':
    main()
