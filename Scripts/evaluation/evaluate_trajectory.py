#!/usr/bin/env python3
import argparse
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from scipy.spatial.transform import Rotation as R
import sys

def load_tum_trajectory(path):
    """Load TUM format trajectory: timestamp x y z qx qy qz qw"""
    data = np.loadtxt(path)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    # Sort by timestamp
    data = data[data[:, 0].argsort()]
    return data

def load_kitti_oxts(oxts_file):
    """Load KITTI Oxts data and convert to local Cartesian coordinates."""
    path = Path(oxts_file)
    if path.is_dir():
        files = sorted(path.glob("*.txt"))
        data = []
        for f in files:
            data.append(np.loadtxt(f))
        data = np.array(data)
    else:
        data = np.loadtxt(path)
    
    if data.ndim == 1:
        data = data.reshape(1, -1)

    # Extract lat, lon, alt (first 3 columns)
    lat = data[:, 0]
    lon = data[:, 1]
    alt = data[:, 2]
    
    # Convert to local Cartesian (Flat Earth approximation around start)
    R_earth = 6378137.0
    lat0 = np.deg2rad(lat[0])
    lon0 = np.deg2rad(lon[0])
    alt0 = alt[0]
    
    x = R_earth * np.cos(lat0) * np.deg2rad(lon - data[0, 1])
    y = R_earth * np.deg2rad(lat - data[0, 0])
    z = alt - alt0
    
    return np.column_stack((x, y, z))

def load_kitti_times(oxts_path):
    """Try to find times.txt to associate timestamps."""
    path = Path(oxts_path)
    candidates = [
        path.parent / "times.txt",
        path.parent.parent / "times.txt", 
        path / "times.txt" 
    ]
    
    for c in candidates:
        if c.exists():
            return np.loadtxt(c)
            
    print("[WARN] times.txt not found, assuming 10Hz")
    if path.is_dir():
        n = len(list(path.glob("*.txt")))
    else:
        n = np.loadtxt(path).shape[0]
    return np.arange(n) * 0.1

def align_trajectories(gt_xyz, pred_data, gt_times):
    """Align prediction to ground truth using timestamps and Umeyama."""
    # Associate Data
    start_t = gt_times[0]
    
    matched_gt = []
    matched_pred = []
    
    for i in range(len(pred_data)):
        t = pred_data[i, 0]
        # Find nearest GT index
        idx = np.searchsorted(gt_times, t)
        if idx > 0 and idx < len(gt_times):
            dt1 = abs(gt_times[idx] - t)
            dt2 = abs(gt_times[idx-1] - t)
            best_idx = idx if dt1 < dt2 else idx-1
            
            if abs(gt_times[best_idx] - t) < 0.2: # 200ms tollerance
                matched_gt.append(gt_xyz[best_idx])
                matched_pred.append(pred_data[i, 1:4])
    
    if not matched_gt:
        print("No matching timestamps found! Trying to force alignment by index/start...")
        n = min(len(pred_data), len(gt_xyz))
        matched_gt = gt_xyz[:n]
        matched_pred = pred_data[:n, 1:4]
        
    gt = np.array(matched_gt)
    pred = np.array(matched_pred)
    
    if len(gt) == 0:
        return np.array([]), np.array([]), np.array([]), 0.0

    # Zero center
    gt_mean = gt[0]
    gt_centered = gt - gt_mean
    pred_centered = pred - pred[0]
    
    # Align Rotation (Kabsch / Umeyama without scale)
    if len(gt_centered) < 3:
         return gt_centered, pred_centered, np.zeros(len(gt_centered)), 0.0

    H = pred_centered.T @ gt_centered
    U, S, Vt = np.linalg.svd(H)
    R_align = Vt.T @ U.T
    
    if np.linalg.det(R_align) < 0:
        Vt[2, :] *= -1
        R_align = Vt.T @ U.T
        
    pred_aligned = pred_centered @ R_align.T
    
    error = np.linalg.norm(gt_centered - pred_aligned, axis=1)
    rmse = np.sqrt(np.mean(error**2))
    
    return gt_centered, pred_aligned, error, rmse

def main():
    print("Script Started")
    parser = argparse.ArgumentParser(description="Evaluate Trajectory")
    parser.add_argument("--pred", required=True, help="Optimized Prediction file (TUM format)")
    parser.add_argument("--baseline", help="Baseline Prediction file (TUM format)")
    parser.add_argument("--gt", required=True, help="Ground Truth file/directory (KITTI Oxts)")
    parser.add_argument("--output", required=True, help="Output directory for plots")
    args = parser.parse_args()
    
    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)
    
    # Load Data
    print(f"Loading Optimized Prediction: {args.pred}")
    try:
        pred_data = load_tum_trajectory(args.pred)
    except Exception as e:
        print(f"Error loading prediction: {e}")
        return

    baseline_data = None
    if args.baseline:
        print(f"Loading Baseline Prediction: {args.baseline}")
        try:
            baseline_data = load_tum_trajectory(args.baseline)
        except Exception as e:
            print(f"Error loading baseline: {e}")
            baseline_data = None # Proceed even if baseline load fails

    print(f"Loading Ground Truth: {args.gt}")
    try:
        gt_xyz_raw = load_kitti_oxts(args.gt)
        gt_times = load_kitti_times(args.gt)
    except Exception as e:
        print(f"Error loading GT: {e}")
        return
    
    # Align Optimized
    gt_opt, pred_opt, error_opt, rmse_opt = align_trajectories(gt_xyz_raw, pred_data, gt_times)
    print(f"Optimized RMSE: {rmse_opt:.4f} m")
    
    # Align Baseline if exists
    gt_base, pred_base, error_base, rmse_base = None, None, None, None
    if baseline_data is not None:
        gt_base, pred_base, error_base, rmse_base = align_trajectories(gt_xyz_raw, baseline_data, gt_times)
        print(f"Baseline RMSE: {rmse_base:.4f} m")
        if rmse_base > 0:
            improvement = (rmse_base - rmse_opt) / rmse_base * 100
            print(f"Improvement: {improvement:.2f}%")
        else:
            improvement = 0.0

    # Plot
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 12))
    
    # Top: Trajectories
    ax1.plot(gt_opt[:, 0], gt_opt[:, 1], 'k-', linewidth=2, label='Ground Truth')
    if pred_base is not None and len(pred_base) > 0:
        ax1.plot(pred_base[:, 0], pred_base[:, 1], 'b--', linewidth=1.5, label=f'Baseline (RMSE={rmse_base:.3f}m)')
    ax1.plot(pred_opt[:, 0], pred_opt[:, 1], 'r-', linewidth=1.5, label=f'Optimized (RMSE={rmse_opt:.3f}m)')
    ax1.set_title("Trajectory Comparison (Aligned)")
    ax1.set_xlabel("X (m)")
    ax1.set_ylabel("Y (m)")
    ax1.legend()
    ax1.grid(True)
    ax1.set_xlim(0, 400)
    
    # Bottom: Errors
    if error_base is not None and len(error_base) > 0:
         ax2.plot(error_base, 'b--', alpha=0.7, label='Baseline Error')
    ax2.plot(error_opt, 'r-', alpha=0.7, label='Optimized Error')
    
    ax2.set_title("Absolute Position Error over Time")
    ax2.set_xlabel("Frame Index")
    ax2.set_ylabel("Error (m)")
    ax2.legend()
    ax2.grid(True)
    
    plt.tight_layout()
    save_path = output_dir / "evaluation_result.png"
    plt.savefig(save_path)
    print(f"Plot saved to {save_path}")
    
    # Save text metrics
    with open(output_dir / "metrics.txt", "w") as f:
        f.write(f"Optimized RMSE: {rmse_opt}\n")
        f.write(f"Optimized Mean Error: {np.mean(error_opt)}\n")
        f.write(f"Optimized Max Error: {np.max(error_opt)}\n")
        if rmse_base is not None:
             f.write(f"Baseline RMSE: {rmse_base}\n")
             f.write(f"Baseline Mean Error: {np.mean(error_base)}\n")
             f.write(f"Baseline Max Error: {np.max(error_base)}\n")
             if rmse_base > 0:
                f.write(f"Improvement: {improvement:.2f}%\n")

if __name__ == "__main__":
    main()
