#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import numpy as np
from scipy.spatial.transform import Rotation as SciRot


plt.rcParams["font.sans-serif"] = [
    "Noto Sans CJK SC",
    "Noto Sans CJK JP",
    "AR PL UKai CN",
    "WenQuanYi Zen Hei",
    "SimHei",
    "DejaVu Sans",
]
plt.rcParams["axes.unicode_minus"] = False


_PALETTE = ["#2196F3", "#F44336", "#4CAF50", "#FF9800", "#9C27B0"]
_LINESTYLES = ["-", "--", "-.", ":", (0, (3, 1, 1, 1))]


def load_tum(path: str | Path) -> np.ndarray:
    data = np.loadtxt(path)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    return data[data[:, 0].argsort()]


def load_kitti_oxts(oxts_path: str | Path) -> tuple[np.ndarray, np.ndarray]:
    path = Path(oxts_path)
    if path.is_dir():
        files = sorted(path.glob("*.txt"))
        rows = [np.loadtxt(f) for f in files]
        data = np.array(rows)
    else:
        data = np.loadtxt(path)

    if data.ndim == 1:
        data = data.reshape(1, -1)

    lat = data[:, 0]
    lon = data[:, 1]
    alt = data[:, 2]

    r_earth = 6_378_137.0
    lat0 = math.radians(lat[0])
    x = r_earth * math.cos(lat0) * np.deg2rad(lon - lon[0])
    y = r_earth * np.deg2rad(lat - lat[0])
    z = alt - alt[0]
    xyz = np.column_stack([x, y, z])

    candidates = [path / "times.txt", path.parent / "times.txt", path.parent.parent / "times.txt"]
    times = None
    for candidate in candidates:
        if candidate.exists():
            times = np.loadtxt(candidate)
            break
    if times is None:
        print("[警告] 未找到 times.txt，按 10Hz 假设时间戳")
        times = np.arange(len(xyz), dtype=np.float64) * 0.1

    return xyz, times


def infer_sequence_name(path_str: str | Path) -> str:
    path = Path(path_str)
    if path.is_file():
        return path.stem
    if path.name:
        return path.name
    return str(path)


def associate_positions(
    pred_data: np.ndarray,
    gt_xyz: np.ndarray,
    gt_times: np.ndarray,
    max_dt: float,
) -> tuple[np.ndarray, np.ndarray]:
    matched_gt = []
    matched_pred = []

    for row in pred_data:
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
            matched_gt.append(gt_xyz[best])
            matched_pred.append(row[1:4])

    if not matched_gt:
        print("[警告] 时间戳未匹配成功，回退到按帧序强制对齐")
        n = min(len(pred_data), len(gt_xyz))
        return gt_xyz[:n], pred_data[:n, 1:4]

    return np.array(matched_gt), np.array(matched_pred)


def umeyama_align(src: np.ndarray, dst: np.ndarray) -> np.ndarray:
    h_mat = src.T @ dst
    u_mat, _, vt_mat = np.linalg.svd(h_mat)
    rot = vt_mat.T @ u_mat.T
    if np.linalg.det(rot) < 0:
        vt_mat[2, :] *= -1
        rot = vt_mat.T @ u_mat.T
    return src @ rot.T


def compute_ate(gt_raw: np.ndarray, pred_raw: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray, dict[str, float]]:
    gt_centered = gt_raw - gt_raw[0]
    pred_centered = pred_raw - pred_raw[0]
    pred_aligned = umeyama_align(pred_centered, gt_centered) if len(gt_centered) >= 3 else pred_centered
    err = np.linalg.norm(gt_centered - pred_aligned, axis=1)
    metrics = {
        "ATE_RMSE": float(np.sqrt(np.mean(err**2))),
        "ATE_mean": float(np.mean(err)),
        "ATE_max": float(np.max(err)),
        "ATE_std": float(np.std(err)),
    }
    return pred_aligned, gt_centered, err, metrics


def compute_rpe(pred_raw: np.ndarray, gt_raw: np.ndarray, delta: int) -> tuple[np.ndarray, dict[str, float]]:
    n_frame = min(len(pred_raw), len(gt_raw))
    errors = []
    for idx in range(n_frame - delta):
        nxt = idx + delta
        gt_delta = np.linalg.norm(gt_raw[nxt] - gt_raw[idx])
        pred_delta = np.linalg.norm(pred_raw[nxt] - pred_raw[idx])
        errors.append(abs(gt_delta - pred_delta))

    errors = np.array(errors)
    if len(errors) == 0:
        return errors, {"RPE_RMSE": 0.0, "RPE_mean": 0.0, "RPE_max": 0.0, "RPE_std": 0.0}

    metrics = {
        "RPE_RMSE": float(np.sqrt(np.mean(errors**2))),
        "RPE_mean": float(np.mean(errors)),
        "RPE_max": float(np.max(errors)),
        "RPE_std": float(np.std(errors)),
    }
    return errors, metrics


def read_calib_mat(calib_path: Path) -> dict[str, np.ndarray]:
    data: dict[str, np.ndarray] = {}
    for raw_line in calib_path.read_text().splitlines():
        line = raw_line.strip()
        if not line:
            continue
        if ":" in line:
            key, value = line.split(":", 1)
        else:
            parts = line.split()
            key, value = parts[0], " ".join(parts[1:])
        data[key.strip()] = np.array([float(x) for x in value.strip().split()], dtype=np.float64)
    return data


def load_gt_poses_lidar(oxts_path: Path, calib_path: Path) -> np.ndarray:
    from pykitti import utils

    packets = utils.load_oxts_packets_and_poses([str(oxts_path)])
    calib = read_calib_mat(calib_path)
    if "Tr_imu_velo" not in calib:
        raise KeyError(f"标定文件缺少 Tr_imu_velo: {calib_path}")

    t_imu_velo = np.vstack([calib["Tr_imu_velo"].reshape(3, 4), [0.0, 0.0, 0.0, 1.0]])
    poses = [packet.T_w_imu @ t_imu_velo for packet in packets]
    return np.array(poses)


def tum_to_pose_mats(tum: np.ndarray) -> np.ndarray:
    poses = []
    for row in tum:
        t_vec = row[1:4]
        qx, qy, qz, qw = row[4:8]
        rot = SciRot.from_quat([qx, qy, qz, qw]).as_matrix()
        pose = np.eye(4)
        pose[:3, :3] = rot
        pose[:3, 3] = t_vec
        poses.append(pose)
    return np.array(poses)


def associate_pose_rows(
    pred_tum: np.ndarray,
    gt_times: np.ndarray,
    gt_poses: np.ndarray,
    max_dt: float,
) -> tuple[np.ndarray, np.ndarray]:
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
    for idx in range(1, len(poses)):
        step = np.linalg.norm(poses[idx][:3, 3] - poses[idx - 1][:3, 3])
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
    d_val = 0.5 * (float(np.trace(rot)) - 1.0)
    d_val = max(-1.0, min(1.0, d_val))
    return float(math.acos(d_val))


def translation_error_m(pose_err: np.ndarray) -> float:
    return float(np.linalg.norm(pose_err[:3, 3]))


def compute_kitti_rel_metrics(
    gt_poses: np.ndarray,
    est_poses: np.ndarray,
    lengths: list[float],
    step: int,
) -> dict[str, Any]:
    if len(gt_poses) != len(est_poses):
        raise ValueError("gt_poses 与 est_poses 长度不一致")

    dist = trajectory_distances(gt_poses)
    t_all: list[float] = []
    r_all: list[float] = []
    per_len: dict[float, dict[str, list[float]]] = {length: {"t": [], "r": []} for length in lengths}

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

    per_length: dict[str, dict[str, float]] = {}
    for length in lengths:
        t_vals = per_len[length]["t"]
        r_vals = per_len[length]["r"]
        per_length[str(int(length))] = {
            "Tr_percent": float(np.mean(t_vals)) if t_vals else float("nan"),
            "Rot_deg_per_100m": float(np.mean(r_vals)) if r_vals else float("nan"),
            "n_segments": float(len(t_vals)),
        }

    return {
        "Tr_percent": float(np.mean(t_all)) if t_all else float("nan"),
        "Rot_deg_per_100m": float(np.mean(r_all)) if r_all else float("nan"),
        "n_segments": int(len(t_all)),
        "per_length": per_length,
    }


def parse_traj_entries(values: list[str]) -> list[dict[str, str]]:
    items = []
    for raw in values:
        if ":" not in raw:
            raise ValueError(f"--traj 参数格式错误，应为 '名称:路径'：{raw}")
        name, path = raw.split(":", 1)
        items.append({"name": name.strip(), "path": str(Path(path).expanduser().resolve())})
    return items


def float_to_text(value: float, digits: int = 4) -> str:
    if value is None or not np.isfinite(value):
        return "N/A"
    return f"{value:.{digits}f}"


def percent_to_text(value: float | None, digits: int = 2) -> str:
    if value is None or not np.isfinite(value):
        return "N/A"
    return f"{value:.{digits}f}%"


def gain_percent(base: float, value: float) -> float | None:
    if base is None or value is None or not np.isfinite(base) or base == 0:
        return None
    return (base - value) / base * 100.0


def select_pair_results(results: list[dict[str, Any]]) -> list[dict[str, Any]]:
    pair_results = [result for result in results if result.get("ate_metrics") is not None]
    return pair_results[:2]


def evaluate_common(
    gt_path: str | Path,
    traj_entries: list[dict[str, str]],
    max_dt: float,
    rpe_delta: int,
) -> tuple[np.ndarray, np.ndarray, list[dict[str, Any]], np.ndarray | None]:
    print(f"\n[真值] 加载：{gt_path}")
    gt_xyz_raw, gt_times = load_kitti_oxts(gt_path)
    print(f"[真值] 帧数：{len(gt_xyz_raw)}，时间范围：{gt_times[0]:.2f} ~ {gt_times[-1]:.2f} s")

    results: list[dict[str, Any]] = []
    gt_c_ref = None

    for entry in traj_entries:
        print(f"\n[轨迹] {entry['name']}  ←  {entry['path']}")
        try:
            pred_data = load_tum(entry["path"])
        except Exception as exc:
            print(f"[跳过] 轨迹加载失败：{exc}")
            results.append(
                {
                    "name": entry["name"],
                    "path": entry["path"],
                    "pred_data": None,
                    "pred_aligned": None,
                    "gt_centered": None,
                    "ate_err": None,
                    "rpe_err": None,
                    "ate_metrics": None,
                    "rpe_metrics": None,
                    "matched_frames": 0,
                }
            )
            continue

        print(f"[轨迹] 帧数：{len(pred_data)}，时间范围：{pred_data[0,0]:.2f} ~ {pred_data[-1,0]:.2f} s")
        gt_matched, pred_matched = associate_positions(pred_data, gt_xyz_raw, gt_times, max_dt)
        print(f"[轨迹] 匹配帧数：{len(gt_matched)}")

        if len(gt_matched) < 3:
            print("[跳过] 有效匹配帧不足 3 帧")
            results.append(
                {
                    "name": entry["name"],
                    "path": entry["path"],
                    "pred_data": pred_data,
                    "pred_aligned": None,
                    "gt_centered": None,
                    "ate_err": None,
                    "rpe_err": None,
                    "ate_metrics": None,
                    "rpe_metrics": None,
                    "matched_frames": int(len(gt_matched)),
                }
            )
            continue

        pred_aligned, gt_centered, ate_err, ate_metrics = compute_ate(gt_matched, pred_matched)
        rpe_err, rpe_metrics = compute_rpe(pred_matched, gt_matched, rpe_delta)
        if gt_c_ref is None:
            gt_c_ref = gt_centered

        print(
            f"[轨迹] ATE RMSE = {ate_metrics['ATE_RMSE']:.4f} m，"
            f"ATE 均值 = {ate_metrics['ATE_mean']:.4f} m，"
            f"RPE RMSE = {rpe_metrics['RPE_RMSE']:.4f} m"
        )

        results.append(
            {
                "name": entry["name"],
                "path": entry["path"],
                "pred_data": pred_data,
                "pred_aligned": pred_aligned,
                "gt_centered": gt_centered,
                "ate_err": ate_err,
                "rpe_err": rpe_err,
                "ate_metrics": ate_metrics,
                "rpe_metrics": rpe_metrics,
                "matched_frames": int(len(gt_matched)),
            }
        )

    return gt_xyz_raw, gt_times, results, gt_c_ref


def save_pair_plot(gt_centered: np.ndarray, pair_results: list[dict[str, Any]], output_path: Path) -> None:
    if len(pair_results) < 2 or gt_centered is None:
        return

    baseline = pair_results[0]
    primary = pair_results[1]

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 12))
    ax1.plot(gt_centered[:, 0], gt_centered[:, 1], "k-", linewidth=2.0, label="真值轨迹")
    ax1.plot(
        baseline["pred_aligned"][:, 0],
        baseline["pred_aligned"][:, 1],
        "b--",
        linewidth=1.5,
        label=f"{baseline['name']}（RMSE={baseline['ate_metrics']['ATE_RMSE']:.3f}m）",
    )
    ax1.plot(
        primary["pred_aligned"][:, 0],
        primary["pred_aligned"][:, 1],
        "r-",
        linewidth=1.5,
        label=f"{primary['name']}（RMSE={primary['ate_metrics']['ATE_RMSE']:.3f}m）",
    )
    ax1.set_title("双轨迹对比（对齐后）")
    ax1.set_xlabel("X (m)")
    ax1.set_ylabel("Y (m)")
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    ax1.set_aspect("equal", adjustable="datalim")

    ax2.plot(baseline["ate_err"], "b--", alpha=0.8, label=f"{baseline['name']} 绝对误差")
    ax2.plot(primary["ate_err"], "r-", alpha=0.8, label=f"{primary['name']} 绝对误差")
    ax2.set_title("绝对位置误差随时间变化")
    ax2.set_xlabel("匹配帧序号")
    ax2.set_ylabel("误差 (m)")
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close()
    print(f"[图表] 已保存 → {output_path}")


def save_overview_plot(gt_centered: np.ndarray, results: list[dict[str, Any]], output_path: Path, title: str) -> None:
    if gt_centered is None:
        return

    fig, axes = plt.subplots(3, 1, figsize=(12, 14))
    fig.suptitle(title or "多轨迹联合评估", fontsize=15, fontweight="bold", y=0.98)

    ax1 = axes[0]
    ax1.plot(gt_centered[:, 0], gt_centered[:, 1], "k-", linewidth=2.0, label="真值轨迹", zorder=10)
    for idx, result in enumerate(results):
        if result["pred_aligned"] is None:
            continue
        color = _PALETTE[idx % len(_PALETTE)]
        line_style = _LINESTYLES[idx % len(_LINESTYLES)]
        label = f"{result['name']}（ATE={result['ate_metrics']['ATE_RMSE']:.3f} m）"
        ax1.plot(result["pred_aligned"][:, 0], result["pred_aligned"][:, 1], color=color, linestyle=line_style, linewidth=1.6, label=label)
    ax1.set_title("轨迹对比（Umeyama 对齐）")
    ax1.set_xlabel("X (m)")
    ax1.set_ylabel("Y (m)")
    ax1.legend(fontsize=9)
    ax1.grid(True, alpha=0.3)
    ax1.set_aspect("equal", adjustable="datalim")

    ax2 = axes[1]
    for idx, result in enumerate(results):
        if result["ate_err"] is None:
            continue
        color = _PALETTE[idx % len(_PALETTE)]
        line_style = _LINESTYLES[idx % len(_LINESTYLES)]
        ax2.plot(result["ate_err"], color=color, linestyle=line_style, linewidth=1.2, alpha=0.8, label=result["name"])
    ax2.set_title("ATE 时序误差")
    ax2.set_xlabel("匹配帧序号")
    ax2.set_ylabel("误差 (m)")
    ax2.legend(fontsize=9)
    ax2.grid(True, alpha=0.3)
    ax2.yaxis.set_minor_locator(ticker.AutoMinorLocator())

    ax3 = axes[2]
    for idx, result in enumerate(results):
        if result["rpe_err"] is None or len(result["rpe_err"]) == 0:
            continue
        color = _PALETTE[idx % len(_PALETTE)]
        line_style = _LINESTYLES[idx % len(_LINESTYLES)]
        ax3.plot(result["rpe_err"], color=color, linestyle=line_style, linewidth=1.2, alpha=0.8, label=result["name"])
    ax3.set_title("RPE 时序误差")
    ax3.set_xlabel("帧序号")
    ax3.set_ylabel("相对误差 (m)")
    ax3.legend(fontsize=9)
    ax3.grid(True, alpha=0.3)

    plt.tight_layout(rect=[0, 0, 1, 0.97])
    output_path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close()
    print(f"[图表] 已保存 → {output_path}")


def save_bar_plot(results: list[dict[str, Any]], output_path: Path) -> None:
    valid_results = [result for result in results if result["ate_metrics"] is not None]
    if not valid_results:
        return

    names = [result["name"] for result in valid_results]
    ate_vals = [result["ate_metrics"]["ATE_RMSE"] for result in valid_results]
    rpe_vals = [result["rpe_metrics"]["RPE_RMSE"] for result in valid_results]

    x_axis = np.arange(len(names))
    width = 0.35

    fig, ax = plt.subplots(figsize=(max(7, len(names) * 2), 5))
    bars1 = ax.bar(
        x_axis - width / 2,
        ate_vals,
        width,
        label="ATE RMSE (m)",
        color=[_PALETTE[idx % len(_PALETTE)] for idx in range(len(names))],
        edgecolor="white",
        linewidth=0.7,
    )
    bars2 = ax.bar(
        x_axis + width / 2,
        rpe_vals,
        width,
        label="RPE RMSE (m)",
        color=[_PALETTE[idx % len(_PALETTE)] for idx in range(len(names))],
        alpha=0.55,
        edgecolor="white",
        linewidth=0.7,
        hatch="//",
    )

    for bar in bars1:
        ax.text(
            bar.get_x() + bar.get_width() / 2,
            bar.get_height() + 0.01,
            f"{bar.get_height():.3f}",
            ha="center",
            va="bottom",
            fontsize=8.5,
            fontweight="bold",
        )
    for bar in bars2:
        ax.text(
            bar.get_x() + bar.get_width() / 2,
            bar.get_height() + 0.01,
            f"{bar.get_height():.3f}",
            ha="center",
            va="bottom",
            fontsize=8.5,
        )

    ax.set_xticks(x_axis)
    ax.set_xticklabels(names, fontsize=10)
    ax.set_ylabel("误差 (m)", fontsize=11)
    ax.set_title("ATE / RPE 对比", fontsize=13, fontweight="bold")
    ax.legend(fontsize=10)
    ax.grid(True, axis="y", alpha=0.3)
    ax.set_ylim(0, max(max(ate_vals), max(rpe_vals)) * 1.25)

    plt.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close()
    print(f"[图表] 已保存 → {output_path}")


def write_pair_outputs(
    gt_path: str | Path,
    pair_results: list[dict[str, Any]],
    output_dir: Path,
    pair_summary_name: str,
    pair_json_name: str,
    pair_plot_name: str,
) -> None:
    if len(pair_results) < 2:
        print("[警告] 有效轨迹不足两条，跳过双轨摘要输出")
        return

    baseline = pair_results[0]
    primary = pair_results[1]
    base_rmse = baseline["ate_metrics"]["ATE_RMSE"]
    primary_rmse = primary["ate_metrics"]["ATE_RMSE"]
    improvement = gain_percent(base_rmse, primary_rmse)
    seq_name = infer_sequence_name(gt_path)

    if pair_plot_name:
        save_pair_plot(baseline["gt_centered"], pair_results, output_dir / pair_plot_name)

    metrics_txt_path = output_dir / pair_summary_name if pair_summary_name else None
    metrics_json_path = output_dir / pair_json_name if pair_json_name else None

    if metrics_txt_path:
        lines = [
            "双轨迹评估结果",
            f"序列号: {seq_name}",
            f"真值路径: {Path(gt_path).resolve()}",
            "",
            f"主轨迹名称: {primary['name']}",
            f"主轨迹文件: {primary['path']}",
            f"主轨迹 RMSE: {primary_rmse:.6f}",
            f"主轨迹平均误差: {primary['ate_metrics']['ATE_mean']:.6f}",
            f"主轨迹最大误差: {primary['ate_metrics']['ATE_max']:.6f}",
            "",
            f"基准轨迹名称: {baseline['name']}",
            f"基准轨迹文件: {baseline['path']}",
            f"基准轨迹 RMSE: {base_rmse:.6f}",
            f"基准轨迹平均误差: {baseline['ate_metrics']['ATE_mean']:.6f}",
            f"基准轨迹最大误差: {baseline['ate_metrics']['ATE_max']:.6f}",
        ]
        if improvement is not None:
            lines.extend(["", f"相对基准提升: {improvement:.2f}%"])
        metrics_txt_path.write_text("\n".join(lines) + "\n")
        print(f"[指标] 已保存 → {metrics_txt_path}")

    if metrics_json_path:
        payload = {
            "sequence": seq_name,
            "gt_path": str(Path(gt_path).resolve()),
            "comparison_available": True,
            "baseline": {
                "name": baseline["name"],
                "path": baseline["path"],
                "rmse": float(base_rmse),
                "mean_error": float(baseline["ate_metrics"]["ATE_mean"]),
                "max_error": float(baseline["ate_metrics"]["ATE_max"]),
                "num_samples": int(len(baseline["ate_err"])),
            },
            "optimized": {
                "name": primary["name"],
                "path": primary["path"],
                "rmse": float(primary_rmse),
                "mean_error": float(primary["ate_metrics"]["ATE_mean"]),
                "max_error": float(primary["ate_metrics"]["ATE_max"]),
                "num_samples": int(len(primary["ate_err"])),
            },
            "improvement_percent": float(improvement) if improvement is not None else None,
            "artifacts": {
                "metrics_txt": str(metrics_txt_path.resolve()) if metrics_txt_path else None,
                "metrics_json": str(metrics_json_path.resolve()),
                "plot": str((output_dir / pair_plot_name).resolve()) if pair_plot_name else None,
            },
        }
        metrics_json_path.write_text(json.dumps(payload, ensure_ascii=True, indent=2) + "\n")
        print(f"[指标] 已保存 → {metrics_json_path}")


def write_multi_summary(
    results: list[dict[str, Any]],
    output_path: Path,
    summary_title: str,
) -> None:
    valid_results = [result for result in results if result["ate_metrics"] is not None]
    if not valid_results:
        print("[警告] 没有可用于汇总的有效轨迹")
        return

    lines = ["=" * 68, summary_title, "=" * 68, ""]
    header = f"{'轨迹配置':<24} {'匹配帧数':>8} {'ATE_RMSE':>10} {'ATE均值':>10} {'ATE最大值':>10} {'RPE_RMSE':>10}"
    lines.append(header)
    lines.append("-" * len(header))

    baseline_ate = valid_results[0]["ate_metrics"]["ATE_RMSE"]
    for result in valid_results:
        ate = result["ate_metrics"]
        rpe = result["rpe_metrics"]
        lines.append(
            f"{result['name']:<24} "
            f"{result['matched_frames']:>8d} "
            f"{ate['ATE_RMSE']:>10.4f} "
            f"{ate['ATE_mean']:>10.4f} "
            f"{ate['ATE_max']:>10.4f} "
            f"{rpe['RPE_RMSE']:>10.4f}"
        )

    lines.append("")
    lines.append("相对第一条有效轨迹的提升幅度：")
    for result in valid_results[1:]:
        improvement = gain_percent(baseline_ate, result["ate_metrics"]["ATE_RMSE"])
        if improvement is None:
            continue
        lines.append(f"  {result['name']:<24} ATE 改善 {improvement:+.2f}%")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text("\n".join(lines) + "\n")
    print(f"[指标] 已保存 → {output_path}")


def write_kitti_summary(
    oxts_path: str | Path,
    calib_path: str | Path,
    traj_entries: list[dict[str, str]],
    output_path: Path,
    lengths: list[float],
    step: int,
    max_dt: float,
) -> None:
    gt_poses = load_gt_poses_lidar(Path(oxts_path).resolve(), Path(calib_path).resolve())
    gt_times = np.arange(len(gt_poses), dtype=np.float64) * 0.1

    rows = []
    per_len_blocks: dict[str, dict[str, dict[str, float]]] = {}
    for entry in traj_entries:
        tum = load_tum(entry["path"])
        matched_tum, matched_gt = associate_pose_rows(tum, gt_times, gt_poses, max_dt)
        est_poses = tum_to_pose_mats(matched_tum)
        metrics = compute_kitti_rel_metrics(matched_gt, est_poses, lengths=lengths, step=step)
        rows.append(
            {
                "name": entry["name"],
                "traj": entry["path"],
                "matched_frames": int(len(matched_tum)),
                "Tr_percent": metrics["Tr_percent"],
                "Rot_deg_per_100m": metrics["Rot_deg_per_100m"],
                "n_segments": metrics["n_segments"],
            }
        )
        per_len_blocks[entry["name"]] = metrics["per_length"]

    base = rows[0]
    for idx, row in enumerate(rows):
        if idx == 0:
            row["Tr_gain_percent"] = 0.0
            row["Rot_gain_percent"] = 0.0
        else:
            tr_gain = gain_percent(base["Tr_percent"], row["Tr_percent"])
            rot_gain = gain_percent(base["Rot_deg_per_100m"], row["Rot_deg_per_100m"])
            row["Tr_gain_percent"] = tr_gain if tr_gain is not None else float("nan")
            row["Rot_gain_percent"] = rot_gain if rot_gain is not None else float("nan")

    lines = [
        "=" * 78,
        "KITTI 相对里程计指标（Tracking OXTS + Tr_imu_velo）",
        "=" * 78,
        "",
        f"真值 OXTS: {Path(oxts_path).resolve()}",
        f"标定文件 : {Path(calib_path).resolve()}",
        f"评估段长 : {', '.join(str(int(length)) for length in lengths)} m",
        f"起点步长 : {step} 帧",
        f"时间戳容差: {max_dt:.3f} s",
        "",
    ]

    header = (
        f"{'轨迹名称':<24} {'匹配帧数':>8} {'段数':>8} "
        f"{'Tr.(%)':>10} {'Rot.(deg/100m)':>16} {'Tr 提升':>10} {'Rot 提升':>10}"
    )
    lines.append(header)
    lines.append("-" * len(header))
    for row in rows:
        lines.append(
            f"{row['name']:<24} {row['matched_frames']:>8d} {row['n_segments']:>8d} "
            f"{float_to_text(row['Tr_percent']):>10} {float_to_text(row['Rot_deg_per_100m']):>16} "
            f"{percent_to_text(row['Tr_gain_percent']):>10} {percent_to_text(row['Rot_gain_percent']):>10}"
        )

    lines.append("")
    lines.append("分段详情：")
    for row in rows:
        name = row["name"]
        lines.append(f"  [{name}]")
        lines.append("    段长(m)      Tr.(%)   Rot.(deg/100m)     段数")
        for length in lengths:
            record = per_len_blocks[name][str(int(length))]
            lines.append(
                f"    {int(length):>7d}   {float_to_text(record['Tr_percent']):>10}   "
                f"{float_to_text(record['Rot_deg_per_100m']):>16}   {int(record['n_segments']):>6d}"
            )

    text = "\n".join(lines)
    print(text)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(text + "\n")
    print(f"[指标] 已保存 → {output_path}")


def run_unified_evaluation(
    gt_path: str | Path,
    traj_entries: list[dict[str, str]],
    output_dir: str | Path,
    *,
    calib_path: str | Path | None = None,
    max_dt: float = 0.2,
    rpe_delta: int = 1,
    title: str = "",
    pair_summary_name: str = "",
    pair_json_name: str = "",
    pair_plot_name: str = "",
    summary_name: str = "",
    summary_title: str = "轨迹对比评估结果",
    overview_plot_name: str = "",
    bar_plot_name: str = "",
    kitti_name: str = "",
    lengths: list[float] | None = None,
    step: int = 10,
) -> int:
    if not traj_entries:
        print("[错误] 至少需要一条轨迹", file=sys.stderr)
        return 1

    out_dir = Path(output_dir).expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    _, _, results, gt_c_ref = evaluate_common(gt_path, traj_entries, max_dt, rpe_delta)

    if pair_summary_name or pair_json_name or pair_plot_name:
        write_pair_outputs(gt_path, select_pair_results(results), out_dir, pair_summary_name, pair_json_name, pair_plot_name)

    if summary_name:
        write_multi_summary(results, out_dir / summary_name, summary_title)

    if overview_plot_name and gt_c_ref is not None:
        save_overview_plot(gt_c_ref, results, out_dir / overview_plot_name, title)

    if bar_plot_name:
        save_bar_plot(results, out_dir / bar_plot_name)

    if kitti_name:
        if calib_path is None:
            print("[错误] 需要提供 --calib 才能计算 KITTI Tr/Rot 指标", file=sys.stderr)
            return 1
        write_kitti_summary(
            gt_path,
            calib_path,
            traj_entries,
            out_dir / kitti_name,
            lengths=lengths or [100.0, 200.0, 300.0, 400.0, 500.0, 600.0, 700.0, 800.0],
            step=step,
            max_dt=max_dt,
        )

    print(f"\n[完成] 评估结果已保存至: {out_dir}")
    return 0


def build_main_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="统一轨迹评估脚本：ATE / RPE / 双轨摘要 / KITTI Tr/Rot")
    parser.add_argument("--gt", required=True, help="KITTI OXTS 真值路径")
    parser.add_argument("--calib", default="", help="KITTI 标定文件路径，用于 Tr/Rot 评估")
    parser.add_argument("--traj", action="append", default=[], metavar="NAME:PATH", help="轨迹参数，格式为 名称:路径，可重复传入")
    parser.add_argument("--output-dir", required=True, help="输出目录")
    parser.add_argument("--max-dt", type=float, default=0.2, help="时间戳匹配最大容差（秒）")
    parser.add_argument("--rpe-delta", type=int, default=1, help="RPE 计算的帧间隔")
    parser.add_argument("--title", default="", help="总览图标题")

    parser.add_argument("--pair-summary-name", default="", help="双轨摘要文本文件名，例如 metrics.txt")
    parser.add_argument("--pair-json-name", default="", help="双轨摘要 JSON 文件名，例如 metrics.json")
    parser.add_argument("--pair-plot-name", default="", help="双轨评估图文件名，例如 evaluation_result.png")

    parser.add_argument("--summary-name", default="", help="多轨汇总文本文件名，例如 metrics_joint_backend.txt")
    parser.add_argument("--summary-title", default="轨迹对比评估结果", help="多轨汇总标题")
    parser.add_argument("--overview-plot-name", default="", help="多轨总览图文件名，例如 evaluation_joint_backend.png")
    parser.add_argument("--bar-plot-name", default="", help="柱状图文件名，例如 ablation_bar.png")

    parser.add_argument("--kitti-name", default="", help="KITTI Tr/Rot 文本文件名，例如 metrics_kitti_tr_rot.txt")
    parser.add_argument("--lengths", default="100,200,300,400,500,600,700,800", help="KITTI Tr/Rot 段长列表")
    parser.add_argument("--step", type=int, default=10, help="KITTI Tr/Rot 起点步长（帧）")
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_main_parser()
    args = parser.parse_args(argv)

    try:
        traj_entries = parse_traj_entries(args.traj)
    except ValueError as exc:
        print(f"[错误] {exc}", file=sys.stderr)
        return 1

    lengths = [float(item.strip()) for item in args.lengths.split(",") if item.strip()]
    return run_unified_evaluation(
        args.gt,
        traj_entries,
        args.output_dir,
        calib_path=args.calib or None,
        max_dt=args.max_dt,
        rpe_delta=args.rpe_delta,
        title=args.title,
        pair_summary_name=args.pair_summary_name,
        pair_json_name=args.pair_json_name,
        pair_plot_name=args.pair_plot_name,
        summary_name=args.summary_name,
        summary_title=args.summary_title,
        overview_plot_name=args.overview_plot_name,
        bar_plot_name=args.bar_plot_name,
        kitti_name=args.kitti_name,
        lengths=lengths,
        step=args.step,
    )


def main_legacy_pair(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="兼容旧版双轨评估入口")
    parser.add_argument("--pred", required=True, help="主轨迹文件（TUM）")
    parser.add_argument("--baseline", help="基准轨迹文件（TUM）")
    parser.add_argument("--gt", required=True, help="KITTI OXTS 真值路径")
    parser.add_argument("--output", required=True, help="输出目录")
    args = parser.parse_args(argv)

    traj_entries = []
    if args.baseline:
        traj_entries.append({"name": "Baseline", "path": str(Path(args.baseline).expanduser().resolve())})
    traj_entries.append({"name": "Optimized", "path": str(Path(args.pred).expanduser().resolve())})

    return run_unified_evaluation(
        args.gt,
        traj_entries,
        args.output,
        pair_summary_name="metrics.txt",
        pair_json_name="metrics.json",
        pair_plot_name="evaluation_result.png",
    )


def main_legacy_joint(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="兼容旧版多轨 ATE/RPE 评估入口")
    parser.add_argument("--gt", required=True, help="KITTI OXTS 真值路径")
    parser.add_argument("--traj", action="append", default=[], metavar="NAME:PATH", help="轨迹参数，格式 名称:路径")
    parser.add_argument("--output", default="Results/", help="输出目录")
    parser.add_argument("--max-dt", type=float, default=0.2, help="时间戳匹配最大容差（秒）")
    parser.add_argument("--rpe-delta", type=int, default=1, help="RPE 帧间隔")
    parser.add_argument("--title", default="", help="图标题")
    args = parser.parse_args(argv)

    try:
        traj_entries = parse_traj_entries(args.traj)
    except ValueError as exc:
        print(f"[错误] {exc}", file=sys.stderr)
        return 1

    return run_unified_evaluation(
        args.gt,
        traj_entries,
        args.output,
        max_dt=args.max_dt,
        rpe_delta=args.rpe_delta,
        title=args.title,
        summary_name="metrics_joint_backend.txt",
        summary_title="轨迹对比评估结果",
        overview_plot_name="evaluation_joint_backend.png",
        bar_plot_name="ablation_bar.png",
    )


def main_legacy_kitti(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="兼容旧版 KITTI Tr/Rot 评估入口")
    parser.add_argument("--oxts", required=True, help="KITTI OXTS 真值路径")
    parser.add_argument("--calib", required=True, help="KITTI 标定文件路径")
    parser.add_argument("--traj", action="append", default=[], metavar="NAME:PATH", help="轨迹参数，格式 名称:路径")
    parser.add_argument("--lengths", default="100,200,300,400,500,600,700,800", help="评估段长列表")
    parser.add_argument("--step", type=int, default=10, help="起点步长（帧）")
    parser.add_argument("--max_dt", type=float, default=0.2, help="时间戳匹配最大容差（秒）")
    parser.add_argument("--output", required=True, help="输出文本文件路径")
    args = parser.parse_args(argv)

    try:
        traj_entries = parse_traj_entries(args.traj)
    except ValueError as exc:
        print(f"[错误] {exc}", file=sys.stderr)
        return 1

    lengths = [float(item.strip()) for item in args.lengths.split(",") if item.strip()]
    write_kitti_summary(
        args.oxts,
        args.calib,
        traj_entries,
        Path(args.output).expanduser().resolve(),
        lengths=lengths,
        step=args.step,
        max_dt=args.max_dt,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
