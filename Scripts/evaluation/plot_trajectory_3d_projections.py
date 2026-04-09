#!/usr/bin/env python3
"""Plot GT + multi-trajectory 3D view and XY/XZ/YZ projections.

Example:
python3 Scripts/evaluation/plot_trajectory_3d_projections.py \
  --oxts Data_Tracking/training/data_tracking_oxts/training/oxts/0020.txt \
  --calib Data_Tracking/training/data_tracking_calib/training/calib/0020.txt \
  --traj "Baseline(Offline,NoWeight):Results/0020_results/trajectory_baseline_offline.txt" \
  --traj "JointOffline:Results/0020_results/trajectory_joint.txt" \
  --output Results/0020_results/trajectory_3d_xyz_projection.png
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Dict, List, Tuple

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
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
        nums = [float(x) for x in value.strip().split()]
        data[key.strip()] = np.array(nums, dtype=np.float64)
    return data


def load_gt_positions_lidar(oxts_path: Path, calib_path: Path) -> np.ndarray:
    packets = utils.load_oxts_packets_and_poses([str(oxts_path)])
    calib = read_calib_mat(calib_path)
    if "Tr_imu_velo" not in calib:
        raise KeyError(f"Missing Tr_imu_velo in calibration file: {calib_path}")
    t_imu_velo = np.vstack([calib["Tr_imu_velo"].reshape(3, 4), [0.0, 0.0, 0.0, 1.0]])

    poses = []
    for packet in packets:
        poses.append(packet.T_w_imu @ t_imu_velo)
    poses_np = np.array(poses)
    return poses_np[:, :3, 3]


def load_tum(path: Path) -> np.ndarray:
    data = np.loadtxt(path)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    return data[data[:, 0].argsort()]


def parse_traj_args(values: List[str]) -> List[Tuple[str, Path]]:
    out: List[Tuple[str, Path]] = []
    for raw in values:
        if ":" in raw:
            name, p = raw.split(":", 1)
        else:
            pth = Path(raw)
            name, p = pth.stem, raw
        out.append((name.strip(), Path(p).expanduser().resolve()))
    return out


def associate_by_timestamp(
    pred_tum: np.ndarray,
    gt_times: np.ndarray,
    gt_xyz: np.ndarray,
    max_dt: float,
) -> Tuple[np.ndarray, np.ndarray]:
    matched_pred = []
    matched_gt = []
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
            matched_pred.append(row[1:4])
            matched_gt.append(gt_xyz[best])
    return np.array(matched_pred), np.array(matched_gt)


def rigid_align(src: np.ndarray, dst: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Find R,t such that src @ R.T + t ~= dst."""
    if len(src) < 3:
        return np.eye(3), np.zeros(3)
    src_mean = src.mean(axis=0)
    dst_mean = dst.mean(axis=0)
    src_c = src - src_mean
    dst_c = dst - dst_mean
    h = src_c.T @ dst_c
    u, _, vt = np.linalg.svd(h)
    rot = vt.T @ u.T
    if np.linalg.det(rot) < 0:
        vt[2, :] *= -1
        rot = vt.T @ u.T
    t = dst_mean - src_mean @ rot.T
    return rot, t


def set_axes_equal_3d(ax, x: np.ndarray, y: np.ndarray, z: np.ndarray):
    x_mid = 0.5 * (x.max() + x.min())
    y_mid = 0.5 * (y.max() + y.min())
    z_mid = 0.5 * (z.max() + z.min())
    radius = 0.5 * max(x.max() - x.min(), y.max() - y.min(), z.max() - z.min())
    if radius <= 0:
        radius = 1.0
    ax.set_xlim(x_mid - radius, x_mid + radius)
    ax.set_ylim(y_mid - radius, y_mid + radius)
    ax.set_zlim(z_mid - radius, z_mid + radius)


def main() -> int:
    parser = argparse.ArgumentParser(description="Plot 3D + XY/XZ/YZ projections for trajectories.")
    parser.add_argument("--oxts", required=True, help="KITTI tracking OXTS sequence file, e.g. 0020.txt")
    parser.add_argument("--calib", required=True, help="KITTI tracking calib sequence file, e.g. 0020.txt")
    parser.add_argument(
        "--traj",
        action="append",
        required=True,
        help="Trajectory in 'Name:/path/to/tum.txt' format; use multiple --traj",
    )
    parser.add_argument("--max_dt", type=float, default=0.2, help="Timestamp match tolerance in seconds")
    parser.add_argument(
        "--output",
        default="Results/0020_results/trajectory_3d_xyz_projection.png",
        help="Output figure path",
    )
    args = parser.parse_args()

    gt_xyz = load_gt_positions_lidar(Path(args.oxts).resolve(), Path(args.calib).resolve())
    gt_times = np.arange(len(gt_xyz), dtype=np.float64) * 0.1

    # Shift GT start to origin for easier comparison.
    gt0 = gt_xyz[0].copy()
    gt_plot = gt_xyz - gt0

    traj_items = parse_traj_args(args.traj)
    aligned_tracks = []
    colors = ["#1f77b4", "#d62728", "#2ca02c", "#ff7f0e", "#9467bd"]

    for i, (name, pth) in enumerate(traj_items):
        tum = load_tum(pth)
        pred_all = tum[:, 1:4]
        pred_m, gt_m = associate_by_timestamp(tum, gt_times, gt_xyz, args.max_dt)
        if len(pred_m) < 3:
            # Fallback: just shift by own start.
            pred_aligned = pred_all - pred_all[0]
        else:
            rot, trans = rigid_align(pred_m, gt_m)
            pred_aligned = pred_all @ rot.T + trans - gt0
        aligned_tracks.append((name, pred_aligned, colors[i % len(colors)]))

    fig = plt.figure(figsize=(16, 11))
    gs = fig.add_gridspec(2, 2, hspace=0.25, wspace=0.25)
    ax3d = fig.add_subplot(gs[0, 0], projection="3d")
    ax_xy = fig.add_subplot(gs[0, 1])
    ax_xz = fig.add_subplot(gs[1, 0])
    ax_yz = fig.add_subplot(gs[1, 1])

    # 3D
    ax3d.plot(gt_plot[:, 0], gt_plot[:, 1], gt_plot[:, 2], color="black", lw=2.0, label="Ground Truth")
    for name, xyz, c in aligned_tracks:
        ax3d.plot(xyz[:, 0], xyz[:, 1], xyz[:, 2], color=c, lw=1.5, label=name)
    all_xyz = [gt_plot] + [x for _, x, _ in aligned_tracks]
    all_xyz_np = np.vstack(all_xyz)
    set_axes_equal_3d(ax3d, all_xyz_np[:, 0], all_xyz_np[:, 1], all_xyz_np[:, 2])
    ax3d.set_title("3D Trajectory")
    ax3d.set_xlabel("X (m)")
    ax3d.set_ylabel("Y (m)")
    ax3d.set_zlabel("Z (m)")
    ax3d.legend(loc="best", fontsize=9)

    # XY
    ax_xy.plot(gt_plot[:, 0], gt_plot[:, 1], color="black", lw=2.0, label="Ground Truth")
    for name, xyz, c in aligned_tracks:
        ax_xy.plot(xyz[:, 0], xyz[:, 1], color=c, lw=1.5, label=name)
    ax_xy.set_title("XY Projection")
    ax_xy.set_xlabel("X (m)")
    ax_xy.set_ylabel("Y (m)")
    ax_xy.grid(True, alpha=0.3)
    ax_xy.set_aspect("equal", adjustable="datalim")

    # XZ
    ax_xz.plot(gt_plot[:, 0], gt_plot[:, 2], color="black", lw=2.0, label="Ground Truth")
    for name, xyz, c in aligned_tracks:
        ax_xz.plot(xyz[:, 0], xyz[:, 2], color=c, lw=1.5, label=name)
    ax_xz.set_title("XZ Projection")
    ax_xz.set_xlabel("X (m)")
    ax_xz.set_ylabel("Z (m)")
    ax_xz.grid(True, alpha=0.3)

    # YZ
    ax_yz.plot(gt_plot[:, 1], gt_plot[:, 2], color="black", lw=2.0, label="Ground Truth")
    for name, xyz, c in aligned_tracks:
        ax_yz.plot(xyz[:, 1], xyz[:, 2], color=c, lw=1.5, label=name)
    ax_yz.set_title("YZ Projection")
    ax_yz.set_xlabel("Y (m)")
    ax_yz.set_ylabel("Z (m)")
    ax_yz.grid(True, alpha=0.3)

    # Share one legend on right-top 2D subplot.
    handles, labels = ax_xy.get_legend_handles_labels()
    ax_xy.legend(handles, labels, fontsize=9)

    fig.suptitle("Trajectory Visualization (GT vs Baseline vs JointOffline)", fontsize=14, fontweight="bold")

    out = Path(args.output).expanduser().resolve()
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=180, bbox_inches="tight")
    plt.close(fig)
    print(f"[Saved] {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
