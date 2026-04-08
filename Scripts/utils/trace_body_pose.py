#!/usr/bin/env python3
"""Capture RViz body motion samples from TF or Odometry and summarize jitter."""
from __future__ import annotations

import argparse
import csv
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from tf2_msgs.msg import TFMessage


@dataclass
class PoseSample:
    index: int
    recv_wall: float
    stamp: float
    parent_frame: str
    child_frame: str
    x: float
    y: float
    z: float
    yaw: float


def wrap_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    _, _, yaw = euler_from_quaternion([x, y, z, w])
    return yaw


class PoseTracer:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.samples: List[PoseSample] = []
        self.started_wall = time.monotonic()

        if args.mode == "tf":
            self.sub = rospy.Subscriber(args.topic, TFMessage, self.tf_callback, queue_size=500)
        else:
            self.sub = rospy.Subscriber(args.topic, Odometry, self.odom_callback, queue_size=500)

    def append_sample(
        self,
        stamp: float,
        parent_frame: str,
        child_frame: str,
        x: float,
        y: float,
        z: float,
        yaw: float,
    ) -> None:
        self.samples.append(
            PoseSample(
                index=len(self.samples),
                recv_wall=time.time(),
                stamp=stamp,
                parent_frame=parent_frame,
                child_frame=child_frame,
                x=x,
                y=y,
                z=z,
                yaw=yaw,
            )
        )

    def tf_callback(self, msg: TFMessage) -> None:
        for transform in msg.transforms:
            if transform.header.frame_id != self.args.parent_frame:
                continue
            if transform.child_frame_id != self.args.child_frame:
                continue
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            self.append_sample(
                stamp=transform.header.stamp.to_sec(),
                parent_frame=transform.header.frame_id,
                child_frame=transform.child_frame_id,
                x=float(translation.x),
                y=float(translation.y),
                z=float(translation.z),
                yaw=quaternion_to_yaw(rotation.x, rotation.y, rotation.z, rotation.w),
            )

    def odom_callback(self, msg: Odometry) -> None:
        parent_frame = msg.header.frame_id or self.args.parent_frame
        child_frame = msg.child_frame_id or self.args.child_frame
        if self.args.require_frame_match:
            if parent_frame != self.args.parent_frame or child_frame != self.args.child_frame:
                return

        position = msg.pose.pose.position
        rotation = msg.pose.pose.orientation
        self.append_sample(
            stamp=msg.header.stamp.to_sec(),
            parent_frame=parent_frame,
            child_frame=child_frame,
            x=float(position.x),
            y=float(position.y),
            z=float(position.z),
            yaw=quaternion_to_yaw(rotation.x, rotation.y, rotation.z, rotation.w),
        )

    def wait(self) -> None:
        while not rospy.is_shutdown():
            if self.args.max_samples > 0 and len(self.samples) >= self.args.max_samples:
                break
            if self.args.duration > 0.0 and (time.monotonic() - self.started_wall) >= self.args.duration:
                break
            time.sleep(0.01)


def summarize(samples: List[PoseSample], args: argparse.Namespace) -> str:
    if not samples:
        return "No samples captured. Start rosbag playback or reproduce the RViz jitter while tracing."

    lines: List[str] = []
    lines.append(f"samples={len(samples)} mode={args.mode} topic={args.topic}")
    lines.append(
        f"frame={samples[0].parent_frame}->{samples[0].child_frame} "
        f"stamp_span={samples[-1].stamp - samples[0].stamp:.3f}s"
    )

    step_lengths: List[float] = []
    stamp_dts: List[float] = []
    wall_dts: List[float] = []
    speeds: List[float] = []
    yaw_steps: List[float] = []
    anomalies: List[str] = []
    non_monotonic = 0
    paused = 0

    for prev, cur in zip(samples, samples[1:]):
        dx = cur.x - prev.x
        dy = cur.y - prev.y
        dz = cur.z - prev.z
        step = math.sqrt(dx * dx + dy * dy + dz * dz)
        dt_stamp = cur.stamp - prev.stamp
        dt_wall = cur.recv_wall - prev.recv_wall
        dyaw = abs(wrap_angle(cur.yaw - prev.yaw))

        step_lengths.append(step)
        stamp_dts.append(dt_stamp)
        wall_dts.append(dt_wall)
        yaw_steps.append(dyaw)

        speed = float("inf") if dt_stamp <= 0.0 else step / dt_stamp
        speeds.append(speed)

        flags: List[str] = []
        if dt_stamp <= 0.0:
            non_monotonic += 1
            flags.append("stamp<=0")
        if dt_stamp > args.pause_threshold:
            paused += 1
            flags.append(f"gap={dt_stamp:.3f}s")
        if step > args.jump_threshold:
            flags.append(f"step={step:.3f}m")
        if speed > args.speed_threshold:
            flags.append(f"speed={speed:.3f}m/s")
        if dyaw > args.yaw_threshold:
            flags.append(f"dyaw={math.degrees(dyaw):.1f}deg")
        if flags:
            anomalies.append(
                f"idx={cur.index} stamp={cur.stamp:.3f} pos=({cur.x:.3f},{cur.y:.3f},{cur.z:.3f}) "
                + " ".join(flags)
            )

    def fmt_range(values: List[float]) -> str:
        if not values:
            return "n/a"
        return f"min={min(values):.4f} mean={sum(values)/len(values):.4f} max={max(values):.4f}"

    lines.append(f"stamp_dt {fmt_range(stamp_dts)}")
    lines.append(f"wall_dt  {fmt_range(wall_dts)}")
    lines.append(f"step_m   {fmt_range(step_lengths)}")
    lines.append(f"speed    {fmt_range(speeds)}")
    lines.append(f"yaw_deg  {fmt_range([math.degrees(v) for v in yaw_steps])}")
    lines.append(f"non_monotonic_stamps={non_monotonic} pause_gaps={paused} anomalies={len(anomalies)}")

    if anomalies:
        lines.append("top anomalies:")
        for item in anomalies[: args.max_anomalies]:
            lines.append(f"  {item}")

    return "\n".join(lines)


def write_csv(samples: List[PoseSample], output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["index", "recv_wall", "stamp", "parent_frame", "child_frame", "x", "y", "z", "yaw_rad"])
        for sample in samples:
            writer.writerow(
                [
                    sample.index,
                    f"{sample.recv_wall:.6f}",
                    f"{sample.stamp:.6f}",
                    sample.parent_frame,
                    sample.child_frame,
                    f"{sample.x:.9f}",
                    f"{sample.y:.9f}",
                    f"{sample.z:.9f}",
                    f"{sample.yaw:.9f}",
                ]
            )


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Trace the body motion that RViz follows. "
            "Default mode captures /tf transform camera_init -> body."
        )
    )
    parser.add_argument("--mode", choices=("tf", "odom"), default="tf")
    parser.add_argument("--topic", default=None, help="Override topic. Defaults: /tf or /Odometry.")
    parser.add_argument("--parent-frame", default="camera_init")
    parser.add_argument("--child-frame", default="body")
    parser.add_argument("--duration", type=float, default=10.0, help="Wall-clock capture time in seconds.")
    parser.add_argument("--max-samples", type=int, default=0, help="Stop after N samples. 0 means unlimited.")
    parser.add_argument("--output", type=Path, default=None, help="Optional CSV output path.")
    parser.add_argument("--jump-threshold", type=float, default=0.5, help="Flag step lengths above this in meters.")
    parser.add_argument("--speed-threshold", type=float, default=8.0, help="Flag speeds above this in m/s.")
    parser.add_argument("--pause-threshold", type=float, default=0.2, help="Flag stamp gaps above this in seconds.")
    parser.add_argument("--yaw-threshold", type=float, default=math.radians(10.0), help="Flag yaw jumps above this in radians.")
    parser.add_argument("--max-anomalies", type=int, default=10)
    parser.add_argument(
        "--require-frame-match",
        action="store_true",
        help="In odom mode, discard messages whose frame ids do not match the parent/child filter.",
    )
    return parser


def main() -> None:
    parser = build_arg_parser()
    args = parser.parse_args()

    if args.topic is None:
        args.topic = "/tf" if args.mode == "tf" else "/Odometry"

    rospy.init_node("trace_body_pose", anonymous=True)
    tracer = PoseTracer(args)
    tracer.wait()

    if args.output is not None:
        write_csv(tracer.samples, args.output)
        print(f"csv={args.output}")

    print(summarize(tracer.samples, args))


if __name__ == "__main__":
    main()
