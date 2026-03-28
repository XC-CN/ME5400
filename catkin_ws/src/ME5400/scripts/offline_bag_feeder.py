#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import threading
from typing import Dict, Optional, Set

import rosbag
import rospy
from genpy import Time as GenpyTime
from rosgraph_msgs.msg import Clock

from ME5400.msg import Detection3DArray, TrackedObjectArray


class SemiSyncOfflineBagFeeder:
    def __init__(self):
        self.bag_path = rospy.get_param("~bag_path", "")
        self.lidar_topic = rospy.get_param("~lidar_topic", "/kitti/velo/pointcloud")
        self.det_topic = rospy.get_param("~det_topic", "/detection/lidar_detections")
        self.track_topic = rospy.get_param("~track_topic", "/mctrack/tracked_objects")

        self.require_detection = bool(rospy.get_param("~require_detection", True))
        self.require_tracking = bool(rospy.get_param("~require_tracking", True))

        self.stamp_tolerance = float(rospy.get_param("~stamp_tolerance", 0.06))
        self.timeout_det = float(rospy.get_param("~timeout_det", 4.0))
        self.timeout_track = float(rospy.get_param("~timeout_track", 5.0))
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

        self.det_sub = rospy.Subscriber(self.det_topic, Detection3DArray, self._det_cb, queue_size=50)
        self.track_sub = rospy.Subscriber(self.track_topic, TrackedObjectArray, self._track_cb, queue_size=50)

        self.current_clock_sec = 0.0

        self.total_lidar = 0
        self.det_ok = 0
        self.det_timeout = 0
        self.track_ok = 0
        self.track_timeout = 0
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
        with self.lock:
            if self.current_target_stamp is not None and self._stamp_match(stamp):
                self.det_ready = True

    def _track_cb(self, msg: TrackedObjectArray):
        stamp = msg.header.stamp.to_sec()
        with self.lock:
            if self.current_target_stamp is not None and self._stamp_match(stamp):
                self.track_ready = True

    def _start_frame_wait(self, frame_id: int, stamp_sec: float):
        with self.lock:
            self.current_frame_id = frame_id
            self.current_target_stamp = stamp_sec
            self.det_ready = not self.require_detection
            self.track_ready = not self.require_tracking

    def _finish_frame_wait(self):
        with self.lock:
            self.current_target_stamp = None

    def _wait_frame_done(self, frame_id: int, stamp_sec: float):
        self._start_frame_wait(frame_id, stamp_sec)

        start_wall = time.monotonic()
        det_deadline = start_wall + max(0.0, self.timeout_det)
        track_deadline = start_wall + max(0.0, self.timeout_track)
        dt_sleep = 1.0 / max(1.0, self.wait_poll_hz)
        det_timed_out = False
        track_timed_out = False

        while not rospy.is_shutdown():
            with self.lock:
                det_done = self.det_ready
                track_done = self.track_ready

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

            if det_done and track_done:
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

        self._finish_frame_wait()

    def run(self):
        rospy.loginfo("[offline_bag_feeder] bag_path=%s", self.bag_path)
        rospy.loginfo("[offline_bag_feeder] lidar_topic=%s det_topic=%s track_topic=%s", self.lidar_topic, self.det_topic, self.track_topic)
        rospy.loginfo("[offline_bag_feeder] require_detection=%s require_tracking=%s eps=%.3f", self.require_detection, self.require_tracking, self.stamp_tolerance)

        if self.start_delay > 0.0:
            rospy.loginfo("[offline_bag_feeder] waiting %.1fs for downstream nodes...", self.start_delay)
            time.sleep(self.start_delay)

        with rosbag.Bag(self.bag_path, "r") as bag:
            for topic, msg, bag_t in bag.read_messages():
                if rospy.is_shutdown():
                    break

                if topic in self.skip_topics:
                    continue

                msg_stamp_sec = self._message_stamp_sec(msg, bag_t)
                self._publish_clock(msg_stamp_sec)

                pub = self._topic_publisher(topic, msg)
                pub.publish(msg)

                if topic == self.lidar_topic:
                    self.total_lidar += 1
                    frame_id = self.total_lidar
                    rospy.loginfo("[offline_bag_feeder] frame=%d lidar stamp=%.6f", frame_id, msg_stamp_sec)
                    self._wait_frame_done(frame_id, msg_stamp_sec)

        if self.total_lidar > 0:
            avg_wait = self.total_wait_sec / self.total_lidar
        else:
            avg_wait = 0.0

        rospy.loginfo("[offline_bag_feeder] finished. lidar_frames=%d", self.total_lidar)
        rospy.loginfo("[offline_bag_feeder] detection: ok=%d timeout=%d", self.det_ok, self.det_timeout)
        rospy.loginfo("[offline_bag_feeder] tracking : ok=%d timeout=%d", self.track_ok, self.track_timeout)
        rospy.loginfo("[offline_bag_feeder] avg_wait_per_frame=%.3fs", avg_wait)


if __name__ == "__main__":
    rospy.init_node("offline_bag_feeder")
    feeder = SemiSyncOfflineBagFeeder()
    feeder.run()
