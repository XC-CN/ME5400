#!/usr/bin/env python3
"""Compute KITTI-style relative translation/rotation errors.

Outputs:
- Tr.(%)
- Rot.(deg/100m)

Example:
python3 Scripts/evaluation/evaluate_kitti_tr_rot.py \
  --oxts Data_Tracking/training/data_tracking_oxts/training/oxts/0020.txt \
  --calib Data_Tracking/training/data_tracking_calib/training/calib/0020.txt \
  --traj "Baseline(Offline,NoWeight):Results/0020_results/trajectory_baseline_offline.txt" \
  --traj "JointOffline:Results/0020_results/trajectory_joint.txt" \
  --output Results/0020_results/metrics_kitti_tr_rot_ac.txt
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np
from pykitti import utils
from scipy.spatial.transform import Rotation as SciRot


def read_calib_mat(calib_path: Path) -> Dict[str, np.ndarray]:
    data: Dict[str, np.ndarray] = {}
    for raw_line in calib_path.read_text().splitlines():
        line = raw_line.strip()
        if not line:
            continue
        if ":" in line:
            key, value = line.split(":", 1)
        else:
            parts = line.split()
            key, value = parts[0], " ".join(parts[1:])
        numbers = [float(x) for x in value.strip().split()]
        data[key.strip()] = np.array(numbers, dtype=np.float64)
    return data


def load_gt_poses_lidar(oxts_path: Path, calib_path: Path) -> np.ndarray:
    packets = utils.load_oxts_packets_and_poses([str(oxts_path)])
    calib = read_calib_mat(calib_path)
    if "Tr_imu_velo" not in calib:
        raise KeyError(f"Missing Tr_imu_velo in calibration file: {calib_path}")

    t_imu_velo = np.vstack([calib["Tr_imu_velo"].reshape(3, 4), [0.0, 0.0, 0.0, 1.0]])
    poses = []
    for packet in packets:
        poses.append(packet.T_w_imu @ t_imu_velo)
    return np.array(poses)


def load_tum(path: Path) -> np.ndarray:
    data = np.loadtxt(path)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    data = data[data[:, 0].argsort()]
    return data


def tum_to_pose_mats(tum: np.ndarray) -> np.ndarray:
    poses = []
    for row in tum:
        t = row[1:4]
        qx, qy, qz, qw = row[4:8]
        rot = SciRot.from_quat([qx, qy, qz, qw]).as_matrix()
        tmat = np.eye(4)
        tmat[:3, :3] = rot
        tmat[:3, 3] = t
        poses.append(tmat)
    return np.array(poses)


def associate_by_timestamp(
    pred_tum: np.ndarray,
    gt_times: np.ndarray,
    gt_poses: np.ndarray,
    max_dt: float,
) -> Tuple[np.ndarray, np.ndarray]:
    matched_pred_rows = []
    matched_gt_poses = []

    for row in pred_tum:
        t = float(row[0])
        idx = int(np.searchsorted(gt_times, t))
        if 0 < idx < len(gt_times):
            dt1 = abs(float(gt_times[idx]) - t)
            dt2 = abs(float(gt_times[idx - 1]) - t)
            best = idx if dt1 < dt2 else idx - 1
        elif idx == 0:
            best = 0
        else:
            best = len(gt_times) - 1

        if abs(float(gt_times[best]) - t) <= max_dt:
            matched_pred_rows.append(row)
            matched_gt_poses.append(gt_poses[best])

    return np.array(matched_pred_rows), np.array(matched_gt_poses)


def trajectory_distances(poses: np.ndarray) -> np.ndarray:
    dists = [0.0]
    for i in range(1, len(poses)):
        step = np.linalg.norm(poses[i][:3, 3] - poses[i - 1][:3, 3])
        dists.append(dists[-1] + float(step))
    return np.array(dists)


def last_frame_from_length(dist: np.ndarray, first_idx: int, length_m: float) -> int:
    target = dist[first_idx] + length_m
    idx = int(np.searchsorted(dist, target))
    if idx >= len(dist):
        return -1
    return idx


def rotation_error_rad(pose_err: np.ndarray) -> float:
    rot = pose_err[:3, :3]
    d = 0.5 * (float(np.trace(rot)) - 1.0)
    d = max(-1.0, min(1.0, d))
    return float(math.acos(d))


def translation_error_m(pose_err: np.ndarray) -> float:
    return float(np.linalg.norm(pose_err[:3, 3]))


def compute_kitti_rel_metrics(
    gt_poses: np.ndarray,
    est_poses: np.ndarray,
    lengths: List[float],
    step: int,
) -> Dict[str, object]:
    if len(gt_poses) != len(est_poses):
        raise ValueError("gt_poses and est_poses must have the same length")

    dist = trajectory_distances(gt_poses)
    t_all: List[float] = []
    r_all: List[float] = []
    per_len: Dict[float, Dict[str, List[float]]] = {
        l: {"t": [], "r": []} for l in lengths
    }

    for first in range(0, len(gt_poses), step):
        for length in lengths:
            last = last_frame_from_length(dist, first, length)
            if last < 0 or last >= len(est_poses):
                continue

            d_gt = np.linalg.inv(gt_poses[first]) @ gt_poses[last]
            d_est = np.linalg.inv(est_poses[first]) @ est_poses[last]
            pose_err = np.linalg.inv(d_est) @ d_gt

            t_rel = translation_error_m(pose_err) / length * 100.0
            r_rel = rotation_error_rad(pose_err) / length * 180.0 / math.pi * 100.0

            t_all.append(t_rel)
            r_all.append(r_rel)
            per_len[length]["t"].append(t_rel)
            per_len[length]["r"].append(r_rel)

    out_per_len: Dict[str, Dict[str, float]] = {}
    for l in lengths:
        t_vals = per_len[l]["t"]
        r_vals = per_len[l]["r"]
        out_per_len[str(int(l))] = {
            "Tr_percent": float(np.mean(t_vals)) if t_vals else float("nan"),
            "Rot_deg_per_100m": float(np.mean(r_vals)) if r_vals else float("nan"),
            "n_segments": float(len(t_vals)),
        }

    return {
        "Tr_percent": float(np.mean(t_all)) if t_all else float("nan"),
        "Rot_deg_per_100m": float(np.mean(r_all)) if r_all else float("nan"),
        "n_segments": int(len(t_all)),
        "per_length": out_per_len,
    }


def parse_traj_arg(values: List[str]) -> List[Tuple[str, Path]]:
    items: List[Tuple[str, Path]] = []
    for raw in values:
        if ":" in raw:
            name, path = raw.split(":", 1)
        else:
            p = Path(raw)
            name, path = p.stem, raw
        items.append((name.strip(), Path(path).expanduser().resolve()))
    return items


def main() -> int:
    parser = argparse.ArgumentParser(description="KITTI relative odometry metrics (Tr/Rot)")
    parser.add_argument("--oxts", required=True, help="KITTI tracking oxts sequence txt (e.g., 0020.txt)")
    parser.add_argument("--calib", required=True, help="KITTI tracking calib sequence txt (e.g., 0020.txt)")
    parser.add_argument(
        "--traj",
        action="append",
        required=True,
        help="Trajectory in 'Name:/path/to/tum.txt' format; can be used multiple times",
    )
    parser.add_argument(
        "--lengths",
        default="100,200,300,400,500,600,700,800",
        help="Segment lengths in meters, comma separated",
    )
    parser.add_argument("--step", type=int, default=10, help="Start frame step (default 10)")
    parser.add_argument("--max_dt", type=float, default=0.2, help="Timestamp association tolerance in seconds")
    parser.add_argument("--output", default="", help="Optional output text file")
    args = parser.parse_args()

    lengths = [float(x.strip()) for x in args.lengths.split(",") if x.strip()]
    traj_items = parse_traj_arg(args.traj)

    gt_poses = load_gt_poses_lidar(Path(args.oxts).resolve(), Path(args.calib).resolve())
    gt_times = np.arange(len(gt_poses), dtype=np.float64) * 0.1

    rows = []
    per_len_blocks = {}

    for name, traj_path in traj_items:
        tum = load_tum(traj_path)
        matched_tum, matched_gt = associate_by_timestamp(tum, gt_times, gt_poses, args.max_dt)
        est_poses = tum_to_pose_mats(matched_tum)
        metrics = compute_kitti_rel_metrics(matched_gt, est_poses, lengths=lengths, step=args.step)

        rows.append(
            {
                "name": name,
                "traj": str(traj_path),
                "matched_frames": int(len(matched_tum)),
                "Tr_percent": metrics["Tr_percent"],
                "Rot_deg_per_100m": metrics["Rot_deg_per_100m"],
                "n_segments": metrics["n_segments"],
            }
        )
        per_len_blocks[name] = metrics["per_length"]

    # improvement vs first trajectory
    base = rows[0]
    for i, row in enumerate(rows):
        if i == 0:
            row["Tr_gain_percent"] = 0.0
            row["Rot_gain_percent"] = 0.0
            continue
        row["Tr_gain_percent"] = (base["Tr_percent"] - row["Tr_percent"]) / base["Tr_percent"] * 100.0
        row["Rot_gain_percent"] = (
            (base["Rot_deg_per_100m"] - row["Rot_deg_per_100m"]) / base["Rot_deg_per_100m"] * 100.0
        )

    lines = []
    lines.append("=" * 78)
    lines.append("KITTI Relative Odometry Metrics (Tracking OXTS + Tr_imu_velo)")
    lines.append("=" * 78)
    lines.append("")
    lines.append(f"GT OXTS: {Path(args.oxts).resolve()}")
    lines.append(f"Calib  : {Path(args.calib).resolve()}")
    lines.append(f"Lengths(m): {', '.join(str(int(l)) for l in lengths)}")
    lines.append(f"Step frames: {args.step}")
    lines.append(f"Timestamp tolerance: {args.max_dt:.3f}s")
    lines.append("")

    header = (
        f"{'Name':<28} {'Matched':>7} {'Segments':>9} "
        f"{'Tr.(%)':>10} {'Rot.(deg/100m)':>16} {'Tr Gain':>10} {'Rot Gain':>10}"
    )
    lines.append(header)
    lines.append("-" * len(header))
    for row in rows:
        lines.append(
            f"{row['name']:<28} {row['matched_frames']:>7d} {row['n_segments']:>9d} "
            f"{row['Tr_percent']:>10.4f} {row['Rot_deg_per_100m']:>16.4f} "
            f"{row['Tr_gain_percent']:>9.2f}% {row['Rot_gain_percent']:>9.2f}%"
        )

    lines.append("")
    lines.append("Per-length details:")
    for row in rows:
        name = row["name"]
        lines.append(f"  [{name}]")
        lines.append("    length(m)   Tr.(%)   Rot.(deg/100m)   n_segments")
        for l in lengths:
            rec = per_len_blocks[name][str(int(l))]
            n_seg = int(rec["n_segments"])
            lines.append(
                f"    {int(l):>8d}   {rec['Tr_percent']:>7.4f}   {rec['Rot_deg_per_100m']:>14.4f}   {n_seg:>10d}"
            )

    text = "\n".join(lines)
    print(text)

    if args.output:
        out = Path(args.output).expanduser().resolve()
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(text + "\n")
        print(f"\n[Saved] {out}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
