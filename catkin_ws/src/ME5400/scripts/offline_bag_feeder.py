#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import threading
from collections import deque
from typing import Deque, Dict, Optional, Set, Tuple

import rosbag
import rospy
from genpy import Time as GenpyTime
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock

from ME5400.msg import Detection3DArray, TrackedObjectArray


class SemiSyncOfflineBagFeeder:
    def __init__(self):
        self.bag_path = rospy.get_param("~bag_path", "")
        self.lidar_topic = rospy.get_param("~lidar_topic", "/kitti/velo/pointcloud")
        self.gated_lidar_topic = rospy.get_param("~gated_lidar_topic", "")
        self.det_topic = rospy.get_param("~det_topic", "/detection/lidar_detections")
        self.track_topic = rospy.get_param("~track_topic", "/mctrack/tracked_objects")
        self.odom_topic = rospy.get_param("~odom_topic", "/joint_backend/odom")
        self.pose_ready_topic = rospy.get_param("~pose_ready_topic", "/mctrack/lidar_pose")

        self.require_detection = bool(rospy.get_param("~require_detection", True))
        self.require_tracking = bool(rospy.get_param("~require_tracking", True))
        self.require_odom = bool(rospy.get_param("~require_odom", False))

        self.stamp_tolerance = float(rospy.get_param("~stamp_tolerance", 0.06))
        self.timeout_det = float(rospy.get_param("~timeout_det", 4.0))
        self.timeout_track = float(rospy.get_param("~timeout_track", 5.0))
        self.timeout_odom = float(rospy.get_param("~timeout_odom", self.timeout_track))
        self.wait_poll_hz = float(rospy.get_param("~wait_poll_hz", 200.0))

        self.publish_clock = bool(rospy.get_param("~publish_clock", True))
        self.clock_topic = rospy.get_param("~clock_topic", "/clock")
        self.clock_step_sec = float(rospy.get_param("~clock_step_sec", 0.002))
        self.auto_set_use_sim_time = bool(rospy.get_param("~auto_set_use_sim_time", True))

        self.skip_topics: Set[str] = set(rospy.get_param("~skip_topics", ["/clock"]))

        self.pub_queue_size = int(rospy.get_param("~pub_queue_size", 50))
        self.start_delay = float(rospy.get_param("~start_delay", 1.5))

        if not self.bag_path:
            raise RuntimeError("~bag_path 不能为空")

        if self.auto_set_use_sim_time and self.publish_clock:
            rospy.set_param("/use_sim_time", True)

        self.publishers: Dict[str, rospy.Publisher] = {}
        self.clock_pub = rospy.Publisher(self.clock_topic, Clock, queue_size=100) if self.publish_clock else None

        self.lock = threading.Lock()
        self.current_target_stamp: Optional[float] = None
        self.current_frame_id: int = -1
        self.det_ready = False
        self.track_ready = False
        self.odom_ready = False
        self.pose_ready = False
        self.det_seen = False
        self.track_seen = False
        self.odom_seen = False

        self.det_sub = rospy.Subscriber(self.det_topic, Detection3DArray, self._det_cb, queue_size=50)
        self.track_sub = rospy.Subscriber(self.track_topic, TrackedObjectArray, self._track_cb, queue_size=50)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self._odom_cb, queue_size=50)
        self.pose_sub = rospy.Subscriber(self.pose_ready_topic, PoseStamped, self._pose_cb, queue_size=20)

        self.current_clock_sec = 0.0

        self.total_lidar = 0
        self.total_gated_lidar = 0
        self.det_ok = 0
        self.det_timeout = 0
        self.track_ok = 0
        self.track_timeout = 0
        self.odom_ok = 0
        self.odom_timeout = 0
        self.waited_frames = 0
        self.total_wait_sec = 0.0

    @staticmethod
    def _stamp_to_sec(stamp) -> float:
        if isinstance(stamp, GenpyTime):
            return stamp.to_sec()
        if hasattr(stamp, "to_sec"):
            return stamp.to_sec()
        return float(stamp)

    def _message_stamp_sec(self, msg, bag_t: GenpyTime) -> float:
        if hasattr(msg, "header") and hasattr(msg.header, "stamp"):
            sec = self._stamp_to_sec(msg.header.stamp)
            if sec > 0.0:
                return sec
        return self._stamp_to_sec(bag_t)

    def _topic_publisher(self, topic: str, msg):
        pub = self.publishers.get(topic)
        if pub is None:
            pub = rospy.Publisher(topic, msg.__class__, queue_size=self.pub_queue_size)
            self.publishers[topic] = pub
            # 使用墙钟睡眠，避免 use_sim_time 下因 /clock 尚未推进导致阻塞
            time.sleep(0.001)
        return pub

    def _publish_clock(self, stamp_sec: float):
        if not self.clock_pub:
            return
        if stamp_sec < self.current_clock_sec:
            stamp_sec = self.current_clock_sec
        self.current_clock_sec = stamp_sec
        clk = Clock()
        clk.clock = rospy.Time.from_sec(self.current_clock_sec)
        self.clock_pub.publish(clk)

    def _tick_clock(self):
        if not self.clock_pub:
            return
        self.current_clock_sec += max(1e-4, self.clock_step_sec)
        clk = Clock()
        clk.clock = rospy.Time.from_sec(self.current_clock_sec)
        self.clock_pub.publish(clk)

    def _stamp_match(self, msg_stamp: float) -> bool:
        if self.current_target_stamp is None:
            return False
        return abs(msg_stamp - self.current_target_stamp) <= self.stamp_tolerance

    def _det_cb(self, msg: Detection3DArray):
        stamp = msg.header.stamp.to_sec()
        if not self.det_seen:
            self.det_seen = True
            rospy.loginfo("[offline_bag_feeder] first detection observed at %.6f", stamp)
        with self.lock:
            if self.current_target_stamp is not None and self._stamp_match(stamp):
                self.det_ready = True

    def _track_cb(self, msg: TrackedObjectArray):
        stamp = msg.header.stamp.to_sec()
        if not self.track_seen:
            self.track_seen = True
            rospy.loginfo("[offline_bag_feeder] first tracking observed at %.6f", stamp)
        with self.lock:
            if self.current_target_stamp is not None and self._stamp_match(stamp):
                self.track_ready = True

    def _odom_cb(self, msg: Odometry):
        stamp = msg.header.stamp.to_sec()
        if not self.odom_seen:
            self.odom_seen = True
            rospy.loginfo("[offline_bag_feeder] first odom observed at %.6f", stamp)
        with self.lock:
            if self.current_target_stamp is not None and self._stamp_match(stamp):
                self.odom_ready = True

    def _pose_cb(self, _msg: PoseStamped):
        self.pose_ready = True

    def _start_frame_wait(self, frame_id: int, stamp_sec: float):
        with self.lock:
            self.current_frame_id = frame_id
            self.current_target_stamp = stamp_sec
            self.det_ready = not self.require_detection
            self.track_ready = not self.require_tracking
            self.odom_ready = not self.require_odom

    def _startup_ready_for_wait(self) -> bool:
        if self.require_detection and not self.det_seen:
            return False
        if self.require_tracking and not self.track_seen:
            return False
        if self.require_odom and not self.odom_seen:
            return False
        return True

    def _finish_frame_wait(self):
        with self.lock:
            self.current_target_stamp = None

    def _wait_frame_done(self, frame_id: int, stamp_sec: float):
        self._start_frame_wait(frame_id, stamp_sec)
        self.waited_frames += 1

        start_wall = time.monotonic()
        det_deadline = start_wall + max(0.0, self.timeout_det)
        track_deadline = start_wall + max(0.0, self.timeout_track)
        odom_deadline = start_wall + max(0.0, self.timeout_odom)
        dt_sleep = 1.0 / max(1.0, self.wait_poll_hz)
        det_timed_out = False
        track_timed_out = False
        odom_timed_out = False

        while not rospy.is_shutdown():
            with self.lock:
                det_done = self.det_ready
                track_done = self.track_ready
                odom_done = self.odom_ready

            now_wall = time.monotonic()

            if self.require_detection and (not det_done) and now_wall >= det_deadline:
                self.det_timeout += 1
                det_done = True
                det_timed_out = True
                with self.lock:
                    self.det_ready = True
                rospy.logwarn("[offline_bag_feeder] frame=%d detection timeout (%.2fs)", frame_id, self.timeout_det)

            if self.require_tracking and (not track_done) and now_wall >= track_deadline:
                self.track_timeout += 1
                track_done = True
                track_timed_out = True
                with self.lock:
                    self.track_ready = True
                rospy.logwarn("[offline_bag_feeder] frame=%d tracking timeout (%.2fs)", frame_id, self.timeout_track)

            if self.require_odom and (not odom_done) and now_wall >= odom_deadline:
                self.odom_timeout += 1
                odom_done = True
                odom_timed_out = True
                with self.lock:
                    self.odom_ready = True
                rospy.logwarn("[offline_bag_feeder] frame=%d odom timeout (%.2fs)", frame_id, self.timeout_odom)

            if det_done and track_done and odom_done:
                break

            self._tick_clock()
            time.sleep(dt_sleep)

        wait_cost = time.monotonic() - start_wall
        self.total_wait_sec += wait_cost

        with self.lock:
            if self.require_detection and self.det_ready and (not det_timed_out):
                self.det_ok += 1
            if self.require_tracking and self.track_ready and (not track_timed_out):
                self.track_ok += 1
            if self.require_odom and self.odom_ready and (not odom_timed_out):
                self.odom_ok += 1

        self._finish_frame_wait()

    def _publish_topic_message(self, topic: str, msg, stamp_sec: float):
        self._publish_clock(stamp_sec)
        pub = self._topic_publisher(topic, msg)
        pub.publish(msg)

    def _gate_lidar_frame(self, frame_id: int, stamp_sec: float, msg):
        gate_topic = self.gated_lidar_topic if self.gated_lidar_topic else self.lidar_topic
        self._publish_topic_message(gate_topic, msg, stamp_sec)
        self.total_gated_lidar += 1
        rospy.loginfo(
            "[offline_bag_feeder] gated_frame=%d lidar stamp=%.6f topic=%s",
            frame_id,
            stamp_sec,
            gate_topic,
        )

    def run(self):
        rospy.loginfo("[offline_bag_feeder] bag_path=%s", self.bag_path)
        rospy.loginfo(
            "[offline_bag_feeder] lidar_topic=%s gated_lidar_topic=%s det_topic=%s track_topic=%s odom_topic=%s pose_ready_topic=%s",
            self.lidar_topic,
            self.gated_lidar_topic if self.gated_lidar_topic else self.lidar_topic,
            self.det_topic,
            self.track_topic,
            self.odom_topic,
            self.pose_ready_topic,
        )
        rospy.loginfo(
            "[offline_bag_feeder] require_detection=%s require_tracking=%s require_odom=%s eps=%.3f",
            self.require_detection,
            self.require_tracking,
            self.require_odom,
            self.stamp_tolerance,
        )

        if self.start_delay > 0.0:
            rospy.loginfo("[offline_bag_feeder] waiting %.1fs for downstream nodes...", self.start_delay)
            time.sleep(self.start_delay)

        split_gated_lidar = bool(self.gated_lidar_topic) and self.gated_lidar_topic != self.lidar_topic
        active_gated_frame: Optional[Tuple[int, float]] = None
        queued_gated_frames: Deque[Tuple[int, float, object]] = deque()

        with rosbag.Bag(self.bag_path, "r") as bag:
            for topic, msg, bag_t in bag.read_messages():
                if rospy.is_shutdown():
                    break

                if topic in self.skip_topics:
                    continue

                msg_stamp_sec = self._message_stamp_sec(msg, bag_t)

                if topic == self.lidar_topic:
                    upcoming_frame_id = self.total_lidar + 1

                    # 分离 raw / gated LiDAR：FAST-LIO 持续读取原始流，PointPillars 仅读取 gated 流。
                    # 由于 FAST-LIO 需要下一拍 IMU 才能完成当前 scan 的去畸变，gated 流必须比 raw 流滞后一帧。
                    if split_gated_lidar and active_gated_frame is not None and upcoming_frame_id >= active_gated_frame[0] + 2:
                        if self._startup_ready_for_wait():
                            self._wait_frame_done(active_gated_frame[0], active_gated_frame[1])
                        active_gated_frame = None
                        if queued_gated_frames:
                            next_frame_id, next_stamp, next_msg = queued_gated_frames.popleft()
                            self._gate_lidar_frame(next_frame_id, next_stamp, next_msg)
                            active_gated_frame = (next_frame_id, next_stamp)

                    self._publish_topic_message(topic, msg, msg_stamp_sec)
                    self.total_lidar += 1
                    frame_id = self.total_lidar
                    rospy.loginfo("[offline_bag_feeder] raw_frame=%d lidar stamp=%.6f", frame_id, msg_stamp_sec)

                    if split_gated_lidar:
                        queued_gated_frames.append((frame_id, msg_stamp_sec, msg))
                        if active_gated_frame is None and queued_gated_frames:
                            if not self.pose_ready:
                                rospy.loginfo("[offline_bag_feeder] waiting for pose_ready... queuing gated frames from frame=%d", frame_id)
                            else:
                                rospy.loginfo("[offline_bag_feeder] pose ready, start gated lidar feed from frame=%d", frame_id)
                            first_frame_id, first_stamp, first_msg = queued_gated_frames.popleft()
                            self._gate_lidar_frame(first_frame_id, first_stamp, first_msg)
                            active_gated_frame = (first_frame_id, first_stamp)
                    else:
                        if active_gated_frame is not None and self._startup_ready_for_wait():
                            self._wait_frame_done(active_gated_frame[0], active_gated_frame[1])
                        active_gated_frame = (frame_id, msg_stamp_sec)
                    continue

                self._publish_topic_message(topic, msg, msg_stamp_sec)

        if not rospy.is_shutdown():
            if split_gated_lidar:
                if active_gated_frame is not None and self._startup_ready_for_wait():
                    self._wait_frame_done(active_gated_frame[0], active_gated_frame[1])
                    active_gated_frame = None
                while len(queued_gated_frames) > 1 and self._startup_ready_for_wait():
                    next_frame_id, next_stamp, next_msg = queued_gated_frames.popleft()
                    self._gate_lidar_frame(next_frame_id, next_stamp, next_msg)
                    self._wait_frame_done(next_frame_id, next_stamp)
                if queued_gated_frames:
                    skipped_frame_id, skipped_stamp, _ = queued_gated_frames.popleft()
                    rospy.logwarn(
                        "[offline_bag_feeder] skip final gated lidar frame=%d stamp=%.6f due to insufficient IMU lookahead",
                        skipped_frame_id,
                        skipped_stamp,
                    )
            elif active_gated_frame is not None and self._startup_ready_for_wait():
                self._wait_frame_done(active_gated_frame[0], active_gated_frame[1])

        if self.waited_frames > 0:
            avg_wait = self.total_wait_sec / self.waited_frames
        else:
            avg_wait = 0.0

        rospy.loginfo("[offline_bag_feeder] finished. lidar_frames=%d", self.total_lidar)
        rospy.loginfo("[offline_bag_feeder] gated_lidar_frames=%d", self.total_gated_lidar)
        rospy.loginfo("[offline_bag_feeder] detection: ok=%d timeout=%d", self.det_ok, self.det_timeout)
        rospy.loginfo("[offline_bag_feeder] tracking : ok=%d timeout=%d", self.track_ok, self.track_timeout)
        rospy.loginfo("[offline_bag_feeder] odom     : ok=%d timeout=%d", self.odom_ok, self.odom_timeout)
        rospy.loginfo("[offline_bag_feeder] avg_wait_per_frame=%.3fs", avg_wait)


if __name__ == "__main__":
    rospy.init_node("offline_bag_feeder")
    feeder = SemiSyncOfflineBagFeeder()
    feeder.run()
