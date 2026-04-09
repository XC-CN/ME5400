#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
evaluate_joint_backend.py
--------------------------
方案 B 消融实验评估脚本。

支持同时比较多条轨迹（Baseline / FAST-LIO+MCTrack / Joint Backend），
输出 ATE、RPE 指标和论文级对比图表。

用法示例
--------
# 仅评估两条轨迹（基准 vs 联合后端）
python Scripts/evaluation/evaluate_joint_backend.py \
  --gt Data_Tracking/training/oxts/0020 \
  --traj "Baseline:Results/trajectory_baseline.txt" \
  --traj "FAST-LIO+MCTrack:Results/trajectory.txt" \
  --traj "Joint Backend:Results/trajectory_joint.txt" \
  --output Results/

# 不需要联合后端轨迹时，可只传两条
python Scripts/evaluation/evaluate_joint_backend.py \
  --gt Data_Tracking/training/oxts/0020 \
  --traj "Baseline:Results/trajectory_baseline.txt" \
  --traj "FAST-LIO+MCTrack:Results/trajectory.txt" \
  --output Results/
"""

import argparse
import math
import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")          # 无头环境下不弹窗
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
from scipy.spatial.transform import Rotation as R

# 优先使用可用的中文字体，避免中文标题/坐标轴显示为方框。
plt.rcParams["font.sans-serif"] = [
    "Noto Sans CJK SC",
    "Noto Sans CJK JP",
    "AR PL UKai CN",
    "WenQuanYi Zen Hei",
    "SimHei",
    "DejaVu Sans",
]
plt.rcParams["axes.unicode_minus"] = False


# ─────────────────────────────────────────────
#  I/O 工具
# ─────────────────────────────────────────────

def load_tum(path: str) -> np.ndarray:
    """读取 TUM 格式轨迹：timestamp x y z qx qy qz qw"""
    data = np.loadtxt(path)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    data = data[data[:, 0].argsort()]   # 按时间戳排序
    return data


def load_kitti_oxts(oxts_path: str) -> tuple[np.ndarray, np.ndarray]:
    """
    读取 KITTI Oxts 目录（每帧一个 .txt），返回 (xyz [N,3], times [N])。
    若找到 times.txt 则使用，否则假设 10 Hz。
    """
    p = Path(oxts_path)
    if p.is_dir():
        files = sorted(p.glob("*.txt"))
        rows = [np.loadtxt(f) for f in files]
        data = np.array(rows)
    else:
        data = np.loadtxt(p)
    if data.ndim == 1:
        data = data.reshape(1, -1)

    lat = data[:, 0]
    lon = data[:, 1]
    alt = data[:, 2]

    R_earth = 6_378_137.0
    lat0 = math.radians(lat[0])
    x = R_earth * math.cos(lat0) * np.deg2rad(lon - lon[0])
    y = R_earth * np.deg2rad(lat - lat[0])
    z = alt - alt[0]
    xyz = np.column_stack([x, y, z])

    # 时间戳
    candidates = [p / "times.txt", p.parent / "times.txt", p.parent.parent / "times.txt"]
    times = None
    for c in candidates:
        if c.exists():
            times = np.loadtxt(c)
            break
    if times is None:
        print("[WARN] times.txt 未找到，假设 10 Hz")
        times = np.arange(len(xyz)) * 0.1
    return xyz, times


# ─────────────────────────────────────────────
#  时间戳对齐
# ─────────────────────────────────────────────

def associate(pred_data: np.ndarray, gt_xyz: np.ndarray, gt_times: np.ndarray,
              max_dt: float = 0.2) -> tuple[np.ndarray, np.ndarray]:
    """
    用时间戳最近邻匹配 pred 帧与 GT 帧，返回 (matched_gt [M,3], matched_pred [M,3])。
    若无匹配则回退到按帧序对齐。
    """
    matched_gt, matched_pred = [], []
    for i in range(len(pred_data)):
        t = pred_data[i, 0]
        idx = np.searchsorted(gt_times, t)
        if 0 < idx < len(gt_times):
            dt1 = abs(gt_times[idx] - t)
            dt2 = abs(gt_times[idx - 1] - t)
            best = idx if dt1 < dt2 else idx - 1
        elif idx == 0:
            best = 0
        else:
            best = len(gt_times) - 1

        if abs(gt_times[best] - t) <= max_dt:
            matched_gt.append(gt_xyz[best])
            matched_pred.append(pred_data[i, 1:4])

    if len(matched_gt) == 0:
        print("[WARN] 时间戳无匹配，按帧序强制对齐")
        n = min(len(pred_data), len(gt_xyz))
        return gt_xyz[:n], pred_data[:n, 1:4]

    return np.array(matched_gt), np.array(matched_pred)


# ─────────────────────────────────────────────
#  Umeyama 对齐（无尺度）
# ─────────────────────────────────────────────

def umeyama_align(src: np.ndarray, dst: np.ndarray) -> np.ndarray:
    """
    将 src 旋转对齐到 dst（平移至零点后 Kabsch 旋转），返回对齐后的 src。
    dst_centered 和 src_centered 应已减去各自首点。
    """
    H = src.T @ dst
    U, _, Vt = np.linalg.svd(H)
    rot = Vt.T @ U.T
    if np.linalg.det(rot) < 0:
        Vt[2, :] *= -1
        rot = Vt.T @ U.T
    return src @ rot.T


# ─────────────────────────────────────────────
#  ATE
# ─────────────────────────────────────────────

def compute_ate(gt_raw: np.ndarray, pred_raw: np.ndarray) -> tuple[np.ndarray, dict]:
    """
    计算 ATE（绝对轨迹误差）。
    返回 (aligned_pred [N,3], metrics dict)。
    """
    gt_c = gt_raw - gt_raw[0]
    pred_c = pred_raw - pred_raw[0]

    pred_aligned = umeyama_align(pred_c, gt_c) if len(gt_c) >= 3 else pred_c

    err = np.linalg.norm(gt_c - pred_aligned, axis=1)
    metrics = {
        "ATE_RMSE": float(np.sqrt(np.mean(err ** 2))),
        "ATE_mean": float(np.mean(err)),
        "ATE_max":  float(np.max(err)),
        "ATE_std":  float(np.std(err)),
    }
    return pred_aligned, gt_c, err, metrics


# ─────────────────────────────────────────────
#  RPE（相对姿态误差，平移部分）
# ─────────────────────────────────────────────

def compute_rpe(pred_raw: np.ndarray, gt_raw: np.ndarray,
                delta: int = 1) -> tuple[np.ndarray, dict]:
    """
    计算 RPE：每隔 delta 帧计算相对位移误差（仅平移，单位 m）。
    返回 (rpe_errors [M], metrics dict)。
    """
    n = min(len(pred_raw), len(gt_raw))
    errors = []
    for i in range(n - delta):
        j = i + delta
        # GT 相对位移
        gt_delta = np.linalg.norm(gt_raw[j] - gt_raw[i])
        # Pred 相对位移
        pred_delta = np.linalg.norm(pred_raw[j] - pred_raw[i])
        errors.append(abs(gt_delta - pred_delta))

    errors = np.array(errors)
    if len(errors) == 0:
        return errors, {"RPE_RMSE": 0, "RPE_mean": 0, "RPE_max": 0, "RPE_std": 0}

    metrics = {
        "RPE_RMSE": float(np.sqrt(np.mean(errors ** 2))),
        "RPE_mean": float(np.mean(errors)),
        "RPE_max":  float(np.max(errors)),
        "RPE_std":  float(np.std(errors)),
    }
    return errors, metrics


# ─────────────────────────────────────────────
#  绘图
# ─────────────────────────────────────────────

# 配色方案：黑色 GT + 几条对比曲线
_PALETTE = ["#2196F3", "#F44336", "#4CAF50", "#FF9800", "#9C27B0"]
_LINESTYLES = ["-", "--", "-.", ":", (0, (3, 1, 1, 1))]


def plot_overview(gt_c: np.ndarray, results: list[dict], out_dir: Path, title: str = ""):
    """
    绘制三子图版本：
      上：XY 轨迹对比
      中：ATE 时序误差
      下：RPE 时序误差
    """
    fig, axes = plt.subplots(3, 1, figsize=(12, 14))
    fig.suptitle(title or "Joint Backend Ablation Evaluation", fontsize=15, fontweight="bold", y=0.98)

    # ── 轨迹图 ──
    ax1 = axes[0]
    ax1.plot(gt_c[:, 0], gt_c[:, 1], "k-", linewidth=2.0, label="Ground Truth", zorder=10)
    for i, r in enumerate(results):
        c = _PALETTE[i % len(_PALETTE)]
        ls = _LINESTYLES[i % len(_LINESTYLES)]
        if r["pred_aligned"] is not None:
            p = r["pred_aligned"]
            label = f'{r["name"]}  (ATE={r["ate_metrics"]["ATE_RMSE"]:.3f} m)'
            ax1.plot(p[:, 0], p[:, 1], color=c, linestyle=ls, linewidth=1.6, label=label)
    ax1.set_title("Trajectory Comparison (Umeyama Aligned)")
    ax1.set_xlabel("X (m)")
    ax1.set_ylabel("Y (m)")
    ax1.legend(fontsize=9)
    ax1.grid(True, alpha=0.3)
    ax1.set_aspect("equal", adjustable="datalim")

    # ── ATE 时序 ──
    ax2 = axes[1]
    for i, r in enumerate(results):
        c = _PALETTE[i % len(_PALETTE)]
        ls = _LINESTYLES[i % len(_LINESTYLES)]
        if r["ate_err"] is not None:
            ax2.plot(r["ate_err"], color=c, linestyle=ls, linewidth=1.2,
                     alpha=0.8, label=r["name"])
    ax2.set_title("ATE (Absolute Trajectory Error) vs Matched Frame Index")
    ax2.set_xlabel("Matched Frame Index")
    ax2.set_ylabel("误差 (m)")
    ax2.legend(fontsize=9)
    ax2.grid(True, alpha=0.3)
    ax2.yaxis.set_minor_locator(ticker.AutoMinorLocator())

    # ── RPE 时序 ──
    ax3 = axes[2]
    for i, r in enumerate(results):
        c = _PALETTE[i % len(_PALETTE)]
        ls = _LINESTYLES[i % len(_LINESTYLES)]
        if r["rpe_err"] is not None and len(r["rpe_err"]) > 0:
            ax3.plot(r["rpe_err"], color=c, linestyle=ls, linewidth=1.2,
                     alpha=0.8, label=r["name"])
    ax3.set_title("RPE (Relative Translation Error, delta=1 frame) vs Frame Index")
    ax3.set_xlabel("Frame Index")
    ax3.set_ylabel("Relative Error (m)")
    ax3.legend(fontsize=9)
    ax3.grid(True, alpha=0.3)

    plt.tight_layout(rect=[0, 0, 1, 0.97])
    save_path = out_dir / "evaluation_joint_backend.png"
    plt.savefig(save_path, dpi=150, bbox_inches="tight")
    plt.close()
    print(f"[图表] 已保存 → {save_path}")


def plot_bar_comparison(results: list[dict], out_dir: Path):
    """绘制 ATE RMSE 和 RPE RMSE 柱状图（论文消融图）。"""
    names = [r["name"] for r in results if r["ate_metrics"] is not None]
    ate_vals = [r["ate_metrics"]["ATE_RMSE"] for r in results if r["ate_metrics"] is not None]
    rpe_vals = [r["rpe_metrics"]["RPE_RMSE"] for r in results if r["rpe_metrics"] is not None]

    if not names:
        return

    x = np.arange(len(names))
    width = 0.35

    fig, ax = plt.subplots(figsize=(max(7, len(names) * 2), 5))
    bars1 = ax.bar(x - width / 2, ate_vals, width, label="ATE RMSE (m)",
                   color=[_PALETTE[i % len(_PALETTE)] for i in range(len(names))],
                   edgecolor="white", linewidth=0.7)
    bars2 = ax.bar(x + width / 2, rpe_vals, width, label="RPE RMSE (m)",
                   color=[_PALETTE[i % len(_PALETTE)] for i in range(len(names))],
                   alpha=0.55, edgecolor="white", linewidth=0.7, hatch="//")

    # 数值标注
    for bar in bars1:
        ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.01,
                f"{bar.get_height():.3f}", ha="center", va="bottom", fontsize=8.5, fontweight="bold")
    for bar in bars2:
        ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.01,
                f"{bar.get_height():.3f}", ha="center", va="bottom", fontsize=8.5)

    ax.set_xticks(x)
    ax.set_xticklabels(names, fontsize=10)
    ax.set_ylabel("Error (m)", fontsize=11)
    ax.set_title("Ablation: ATE / RPE Comparison Across Configurations", fontsize=13, fontweight="bold")
    ax.legend(fontsize=10)
    ax.grid(True, axis="y", alpha=0.3)
    ax.set_ylim(0, max(max(ate_vals), max(rpe_vals)) * 1.25)

    plt.tight_layout()
    save_path = out_dir / "ablation_bar.png"
    plt.savefig(save_path, dpi=150, bbox_inches="tight")
    plt.close()
    print(f"[图表] 已保存 → {save_path}")


# ─────────────────────────────────────────────
#  保存指标文本
# ─────────────────────────────────────────────

def save_metrics(results: list[dict], out_dir: Path):
    path = out_dir / "metrics_joint_backend.txt"
    lines = ["=" * 60, "联合后端消融实验评估结果", "=" * 60, ""]

    # 表头
    header = f"{'配置':<25} {'ATE_RMSE':>10} {'ATE_mean':>10} {'ATE_max':>10} {'RPE_RMSE':>10} {'RPE_mean':>10}"
    lines.append(header)
    lines.append("-" * len(header))

    baseline_ate = None
    for r in results:
        if r["ate_metrics"] is None:
            continue
        am = r["ate_metrics"]
        rm = r["rpe_metrics"] or {}
        row = (f'{r["name"]:<25} '
               f'{am["ATE_RMSE"]:>10.4f} '
               f'{am["ATE_mean"]:>10.4f} '
               f'{am["ATE_max"]:>10.4f} '
               f'{rm.get("RPE_RMSE", float("nan")):>10.4f} '
               f'{rm.get("RPE_mean", float("nan")):>10.4f}')
        lines.append(row)
        if baseline_ate is None:
            baseline_ate = am["ATE_RMSE"]

    # 提升幅度
    lines.append("")
    lines.append("提升幅度（相对第一条轨迹）：")
    for r in results[1:]:
        if r["ate_metrics"] is None or baseline_ate is None or baseline_ate == 0:
            continue
        imp = (baseline_ate - r["ate_metrics"]["ATE_RMSE"]) / baseline_ate * 100
        lines.append(f'  {r["name"]:<25}  ATE 改善 {imp:+.2f}%')

    lines.append("")
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")
    print(f"[指标] 已保存 → {path}")

    # 同时打印到终端
    print()
    for l in lines:
        print(l)


# ─────────────────────────────────────────────
#  主函数
# ─────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(
        description="方案 B 消融实验评估：ATE + RPE + 轨迹对比图",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    p.add_argument(
        "--gt", required=True,
        help="Ground Truth 路径：KITTI Oxts 目录（每帧一个 .txt）或包含 times.txt 的父目录",
    )
    p.add_argument(
        "--traj", action="append", default=[], metavar="NAME:PATH",
        help=(
            "轨迹文件，格式为 '显示名称:文件路径'（TUM 格式）。"
            "可多次指定以比较多条轨迹，建议第一条为 Baseline。"
        ),
    )
    p.add_argument(
        "--output", default="Results/",
        help="输出目录（默认 Results/）",
    )
    p.add_argument(
        "--max-dt", type=float, default=0.2,
        help="时间戳匹配最大容差（秒，默认 0.2）",
    )
    p.add_argument(
        "--rpe-delta", type=int, default=1,
        help="RPE 计算帧间隔（默认 1）",
    )
    p.add_argument(
        "--title", default="",
        help="图表标题（可选）",
    )
    return p.parse_args()


def main():
    args = parse_args()

    if not args.traj:
        print("[错误] 至少需要用 --traj 指定一条轨迹", file=sys.stderr)
        sys.exit(1)

    # 解析轨迹列表
    traj_list = []
    for entry in args.traj:
        if ":" not in entry:
            print(f"[错误] --traj 格式错误（应为 '名称:路径'）：{entry}", file=sys.stderr)
            sys.exit(1)
        name, path = entry.split(":", 1)
        traj_list.append({"name": name.strip(), "path": path.strip()})

    out_dir = Path(args.output)
    out_dir.mkdir(parents=True, exist_ok=True)

    # 加载 Ground Truth
    print(f"\n[GT] 加载：{args.gt}")
    try:
        gt_xyz_raw, gt_times = load_kitti_oxts(args.gt)
    except Exception as e:
        print(f"[错误] 无法加载 GT：{e}", file=sys.stderr)
        sys.exit(1)
    print(f"     GT 帧数：{len(gt_xyz_raw)}, 时间范围：{gt_times[0]:.2f}~{gt_times[-1]:.2f} s")

    # 处理各条轨迹
    results = []
    gt_c_ref = None   # 用第一条成功匹配的 GT 作为公共参考

    for t in traj_list:
        print(f"\n[轨迹] {t['name']}  ←  {t['path']}")
        try:
            pred_data = load_tum(t["path"])
        except Exception as e:
            print(f"  [跳过] 加载失败：{e}")
            results.append({
                "name": t["name"],
                "pred_aligned": None, "ate_err": None,
                "rpe_err": None, "ate_metrics": None, "rpe_metrics": None,
            })
            continue

        print(f"  帧数：{len(pred_data)}, 时间范围：{pred_data[0,0]:.2f}~{pred_data[-1,0]:.2f} s")

        # 时间戳对齐
        gt_matched, pred_matched = associate(pred_data, gt_xyz_raw, gt_times, args.max_dt)
        print(f"  匹配帧数：{len(gt_matched)}")

        if len(gt_matched) < 3:
            print("  [跳过] 有效匹配帧不足 3 帧")
            results.append({
                "name": t["name"],
                "pred_aligned": None, "ate_err": None,
                "rpe_err": None, "ate_metrics": None, "rpe_metrics": None,
            })
            continue

        # ATE
        pred_aligned, gt_c, ate_err, ate_metrics = compute_ate(gt_matched, pred_matched)
        if gt_c_ref is None:
            gt_c_ref = gt_c

        # RPE
        rpe_err, rpe_metrics = compute_rpe(pred_matched, gt_matched, args.rpe_delta)

        print(f"  ATE RMSE = {ate_metrics['ATE_RMSE']:.4f} m  |  "
              f"ATE mean = {ate_metrics['ATE_mean']:.4f} m  |  "
              f"RPE RMSE = {rpe_metrics['RPE_RMSE']:.4f} m")

        results.append({
            "name": t["name"],
            "pred_aligned": pred_aligned,
            "ate_err": ate_err,
            "rpe_err": rpe_err,
            "ate_metrics": ate_metrics,
            "rpe_metrics": rpe_metrics,
        })

    # 绘图
    print("\n[绘图] 生成图表...")
    if gt_c_ref is not None:
        plot_overview(gt_c_ref, results, out_dir, title=args.title)
    plot_bar_comparison(results, out_dir)
    save_metrics(results, out_dir)

    print("\n[完成] 所有结果已保存至:", out_dir.resolve())


if __name__ == "__main__":
    main()
