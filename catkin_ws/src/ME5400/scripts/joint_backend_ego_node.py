#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from collections import deque
from dataclasses import dataclass
from typing import Dict, Optional

import numpy as np
import rospy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from scipy.optimize import least_squares

from ME5400.msg import TrackedObjectArray


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def quat_to_yaw(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw * 0.5)
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    return q


def relative_pose(a, b):
    """Return relative transform from pose a to b in a's local frame (SE2)."""
    ax, ay, ayaw = a
    bx, by, byaw = b
    dx_w = bx - ax
    dy_w = by - ay
    c = math.cos(ayaw)
    s = math.sin(ayaw)
    dx = c * dx_w + s * dy_w
    dy = -s * dx_w + c * dy_w
    dyaw = wrap_angle(byaw - ayaw)
    return np.array([dx, dy, dyaw], dtype=np.float64)


def world_to_local(ego, obj):
    """Project object world pose to ego local frame (SE2)."""
    ex, ey, eyaw = ego
    ox, oy, oyaw = obj
    dx_w = ox - ex
    dy_w = oy - ey
    c = math.cos(eyaw)
    s = math.sin(eyaw)
    lx = c * dx_w + s * dy_w
    ly = -s * dx_w + c * dy_w
    lyaw = wrap_angle(oyaw - eyaw)
    return np.array([lx, ly, lyaw], dtype=np.float64)


@dataclass
class OdomState:
    stamp: float
    x: float
    y: float
    yaw: float
    cov: np.ndarray
    raw: Odometry


@dataclass
class TrackObs:
    track_id: int
    stamp: float
    score: float
    track_length: int
    world_x: float
    world_y: float
    world_yaw: float
    lidar_x: float
    lidar_y: float
    lidar_yaw: float
    vel_x: float
    vel_y: float
    speed: float
    dv: float


@dataclass
class TrackSnapshot:
    stamp: float
    objects: Dict[int, TrackObs]


class JointBackendEgoNode:
    def __init__(self):
        # Params
        self.window_size = int(rospy.get_param("~window_size", 10))
        self.lambda_obj = float(rospy.get_param("~lambda_obj", 0.3))
        self.score_th = float(rospy.get_param("~score_th", 0.5))
        self.track_len_th = int(rospy.get_param("~track_len_th", 3))
        self.max_dt = float(rospy.get_param("~max_dt", 0.2))
        self.max_dv = float(rospy.get_param("~max_dv", 3.0))
        self.min_obj_weight = float(rospy.get_param("~min_obj_weight", 0.1))
        self.optimize_hz = float(rospy.get_param("~optimize_hz", 10.0))

        fallback = rospy.get_param("~odom_noise_fallback", [0.20, 0.20, 0.10])
        if not isinstance(fallback, list) or len(fallback) != 3:
            fallback = [0.20, 0.20, 0.10]
        self.odom_noise_fallback = np.array([max(1e-3, float(x)) for x in fallback], dtype=np.float64)

        self.odom_topic = rospy.get_param("~odom_topic", "/Odometry")
        self.tracks_topic = rospy.get_param("~tracks_topic", "/mctrack/tracked_objects")
        self.out_topic = rospy.get_param("~out_topic", "/joint_backend/odom")

        self.odom_buffer = deque(maxlen=400)
        self.track_buffer = deque(maxlen=400)
        self.last_track_speed: Dict[int, float] = {}

        self.last_opt_time = 0.0

        self.sub_odom = rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb, queue_size=50)
        self.sub_tracks = rospy.Subscriber(self.tracks_topic, TrackedObjectArray, self.tracks_cb, queue_size=20)
        self.pub_odom = rospy.Publisher(self.out_topic, Odometry, queue_size=20)

        rospy.loginfo("[joint_backend_ego] started: window=%d lambda=%.3f", self.window_size, self.lambda_obj)

    def odom_cb(self, msg: Odometry):
        yaw = quat_to_yaw(msg.pose.pose.orientation)
        cov_msg = msg.pose.covariance
        if len(cov_msg) >= 36:
            sig_x = math.sqrt(max(1e-6, cov_msg[0]))
            sig_y = math.sqrt(max(1e-6, cov_msg[7]))
            sig_yaw = math.sqrt(max(1e-6, cov_msg[35]))
            cov = np.array([sig_x, sig_y, sig_yaw], dtype=np.float64)
            # Guard against invalid/zero covariances
            if (not np.isfinite(cov).all()) or np.any(cov < 1e-3):
                cov = self.odom_noise_fallback.copy()
        else:
            cov = self.odom_noise_fallback.copy()

        state = OdomState(
            stamp=msg.header.stamp.to_sec(),
            x=float(msg.pose.pose.position.x),
            y=float(msg.pose.pose.position.y),
            yaw=float(yaw),
            cov=cov,
            raw=msg,
        )
        self.odom_buffer.append(state)

        now = rospy.Time.now().to_sec()
        if now - self.last_opt_time < 1.0 / max(1.0, self.optimize_hz):
            return
        self.last_opt_time = now

        self.optimize_and_publish()

    def tracks_cb(self, msg: TrackedObjectArray):
        stamp = msg.header.stamp.to_sec()
        objs: Dict[int, TrackObs] = {}

        for o in msg.objects:
            tid = int(o.track_id)
            vx, vy = float(o.velocity_fusion[0]), float(o.velocity_fusion[1])
            speed = math.hypot(vx, vy)
            prev_speed = self.last_track_speed.get(tid, speed)
            dv = abs(speed - prev_speed)
            self.last_track_speed[tid] = speed

            obs = TrackObs(
                track_id=tid,
                stamp=stamp,
                score=float(o.detection_score),
                track_length=int(o.track_length),
                world_x=float(o.pose.position.x),
                world_y=float(o.pose.position.y),
                world_yaw=float(o.yaw),
                lidar_x=float(o.lidar_pose.position.x),
                lidar_y=float(o.lidar_pose.position.y),
                lidar_yaw=float(o.lidar_yaw),
                vel_x=vx,
                vel_y=vy,
                speed=speed,
                dv=dv,
            )
            objs[tid] = obs

        self.track_buffer.append(TrackSnapshot(stamp=stamp, objects=objs))

    def find_nearest_snapshot(self, t: float) -> Optional[TrackSnapshot]:
        if not self.track_buffer:
            return None
        best = None
        best_dt = 1e9
        for s in self.track_buffer:
            dt = abs(s.stamp - t)
            if dt < best_dt:
                best_dt = dt
                best = s
        if best is None or best_dt > self.max_dt:
            return None
        return best

    def build_window(self):
        if len(self.odom_buffer) < self.window_size:
            return None
        return list(self.odom_buffer)[-self.window_size:]

    def optimize_and_publish(self):
        window = self.build_window()
        if window is None:
            return

        n = len(window)
        x0 = np.zeros(3 * n, dtype=np.float64)
        for i, st in enumerate(window):
            x0[3 * i : 3 * i + 3] = np.array([st.x, st.y, st.yaw])

        # Measured odom deltas and weights
        odom_meas = []
        odom_w = []
        for i in range(1, n):
            a = np.array([window[i - 1].x, window[i - 1].y, window[i - 1].yaw], dtype=np.float64)
            b = np.array([window[i].x, window[i].y, window[i].yaw], dtype=np.float64)
            odom_meas.append(relative_pose(a, b))
            sig = 0.5 * (window[i - 1].cov + window[i].cov)
            odom_w.append(1.0 / np.maximum(sig, 1e-3))

        def residual_fn(theta: np.ndarray):
            poses = theta.reshape((-1, 3))
            residuals = []

            # Odom consistency factors
            for i in range(1, n):
                pred = relative_pose(poses[i - 1], poses[i])
                r = pred - odom_meas[i - 1]
                r[2] = wrap_angle(r[2])
                residuals.extend((odom_w[i - 1] * r).tolist())

            # Object observation consistency factors
            for i in range(n):
                t_i = window[i].stamp
                snap = self.find_nearest_snapshot(t_i)
                if snap is None:
                    continue

                for obs in snap.objects.values():
                    if obs.track_length < self.track_len_th:
                        continue
                    if obs.score < self.score_th:
                        continue
                    if obs.dv > self.max_dv:
                        continue

                    # Propagate world reference by velocity to frame timestamp
                    dt = t_i - obs.stamp
                    ow = np.array(
                        [
                            obs.world_x + obs.vel_x * dt,
                            obs.world_y + obs.vel_y * dt,
                            obs.world_yaw,
                        ],
                        dtype=np.float64,
                    )

                    z_pred = world_to_local(poses[i], ow)
                    z_meas = np.array([obs.lidar_x, obs.lidar_y, obs.lidar_yaw], dtype=np.float64)
                    r = z_pred - z_meas
                    r[2] = wrap_angle(r[2])

                    # Object factor weight: score + track length + global lambda
                    score_norm = max(0.0, min(1.0, (obs.score - self.score_th) / max(1e-3, 1.0 - self.score_th)))
                    len_norm = max(0.0, min(1.0, (obs.track_length - self.track_len_th + 1) / 6.0))
                    w_obj = max(self.min_obj_weight, math.sqrt(self.lambda_obj) * (0.5 * score_norm + 0.5 * len_norm))

                    residuals.extend((w_obj * r).tolist())

            return np.array(residuals, dtype=np.float64)

        try:
            r0 = residual_fn(x0)
            if r0.size < 6:
                return
            # lm works well for small dense windows
            sol = least_squares(residual_fn, x0, method="lm", max_nfev=40)
            x_opt = sol.x.reshape((-1, 3))
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "[joint_backend_ego] optimize failed: %s", str(exc))
            return

        # Publish only latest optimized odom
        last_raw = window[-1].raw
        last_opt = x_opt[-1]

        out = Odometry()
        out.header = Header(stamp=last_raw.header.stamp, frame_id=last_raw.header.frame_id)
        out.child_frame_id = last_raw.child_frame_id

        out.pose.pose.position = last_raw.pose.pose.position
        out.pose.pose.position.x = float(last_opt[0])
        out.pose.pose.position.y = float(last_opt[1])
        out.pose.pose.orientation = yaw_to_quat(float(last_opt[2]))
        out.pose.covariance = last_raw.pose.covariance

        out.twist = last_raw.twist

        self.pub_odom.publish(out)


if __name__ == "__main__":
    rospy.init_node("joint_backend_ego_node")
    JointBackendEgoNode()
    rospy.spin()
