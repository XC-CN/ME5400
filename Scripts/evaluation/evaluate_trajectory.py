#!/usr/bin/env python3
import argparse
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from scipy.spatial.transform import Rotation as R

def load_tum_trajectory(path):
    """Load TUM format trajectory: timestamp x y z qx qy qz qw"""
    data = np.loadtxt(path)
    # Sort by timestamp
    data = data[data[:, 0].argsort()]
    return data

def load_kitti_oxts(oxts_file):
    """Load KITTI Oxts data and convert to local Cartesian coordinates."""
    # Assuming oxts_file is a single combined file or we load individual files
    # The user instruction implies a single file path for GT: .../oxts/$SEQ_ID.txt
    # Usually KITTI provides 000000.txt per frame in a folder, or a combined txt.
    # We will support loading a folder of txts or a single txt.
    
    path = Path(oxts_file)
    if path.is_dir():
        files = sorted(path.glob("*.txt"))
        data = []
        for f in files:
            data.append(np.loadtxt(f))
        data = np.array(data)
    else:
        data = np.loadtxt(path)
    
    # Extract lat, lon, alt (first 3 columns)
    # KITTI Oxts: lat, lon, alt, roll, pitch, yaw, ...
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
    
    # Note: KITTI Oxts frame is East-North-Up (ENU) or similar? 
    # Usually we need to check the exact definition. 
    # KITTI Coordinates: x: forward, y: left, z: up (Velodyne)
    # Oxts is GPS/IMU. 
    # Conversion above gives x=East, y=North (approx). 
    # We will rely on alignment to fix rotation.
    
    return np.column_stack((x, y, z))

def load_kitti_times(oxts_path):
    """Try to find times.txt to associate timestamps."""
    # Try to look for times.txt in the parent/sibling directories
    path = Path(oxts_path)
    # Common KITTI structure checks
    candidates = [
        path.parent / "times.txt",
        path.parent.parent / "times.txt", 
        path / "times.txt" # if path is dir
    ]
    
    for c in candidates:
        if c.exists():
            return np.loadtxt(c)
            
    # If not found, create dummy timestamps (10Hz)
    print("[WARN] times.txt not found, assuming 10Hz")
    if path.is_dir():
        n = len(list(path.glob("*.txt")))
    else:
        n = np.loadtxt(path).shape[0]
    return np.arange(n) * 0.1

def align_trajectories(gt_xyz, pred_data):
    """Align prediction to ground truth using timestamps and Umeyama/rigid transform."""
    pred_times = pred_data[:, 0]
    pred_xyz = pred_data[:, 1:4]
    
    # Assuming gt_times matches gt_xyz row-by-row
    # We don't have explicit GT timestamps passed in here, let's fix that structure
    # For now, we align by timestamp association
    pass 

def main():
    parser = argparse.ArgumentParser(description="Evaluate Trajectory")
    parser.add_argument("--pred", required=True, help="Prediction file (TUM format)")
    parser.add_argument("--gt", required=True, help="Ground Truth file/directory (KITTI Oxts)")
    parser.add_argument("--output", required=True, help="Output directory for plots")
    args = parser.parse_args()
    
    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)
    
    # Load Data
    print(f"Loading prediction: {args.pred}")
    try:
        pred_data = load_tum_trajectory(args.pred) # t, x, y, z, ...
    except Exception as e:
        print(f"Error loading prediction: {e}")
        return

    print(f"Loading Ground Truth: {args.gt}")
    gt_xyz_raw = load_kitti_oxts(args.gt)
    gt_times = load_kitti_times(args.gt)
    
    # Associate Data
    # Find nearest GT point for each Pred point based on time
    # Valid range: GT start time to GT end time
    start_t = gt_times[0]
    end_t = gt_times[-1]
    
    matches = []
    
    # FastLIO typically uses relative time from start (0.0) or ROS time
    # If FastLIO uses ROS time (from bag), it should match KITTI times (if bag is based on KITTI times)
    # Check time offset
    time_diff = np.mean(pred_data[:10, 0]) - start_t
    
    print(f"Time range Pred: {pred_data[0,0]:.2f} - {pred_data[-1,0]:.2f}")
    print(f"Time range GT: {gt_times[0]:.2f} - {gt_times[-1]:.2f}")
    
    matched_gt = []
    matched_pred = []
    
    for i in range(len(pred_data)):
        t = pred_data[i, 0]
        # Find nearest GT index
        idx = np.searchsorted(gt_times, t)
        if idx > 0 and idx < len(gt_times):
            # Check closer of idx or idx-1
            dt1 = abs(gt_times[idx] - t)
            dt2 = abs(gt_times[idx-1] - t)
            best_idx = idx if dt1 < dt2 else idx-1
            
            if abs(gt_times[best_idx] - t) < 0.2: # 200ms tollerance
                matched_gt.append(gt_xyz_raw[best_idx])
                matched_pred.append(pred_data[i, 1:4])
                
    if not matched_gt:
        print("No matching timestamps found! Trying to force alignment by index/start...")
        # Fallback: Assume start at same time
        offset = pred_data[0,0] - gt_times[0]
        # Or just take min length
        n = min(len(pred_data), len(gt_xyz_raw))
        matched_gt = gt_xyz_raw[:n]
        matched_pred = pred_data[:n, 1:4]
    
    gt = np.array(matched_gt)
    pred = np.array(matched_pred)
    
    # Zero center
    gt_centered = gt - gt[0]
    pred_centered = pred - pred[0]
    
    # Align Rotation (Kabsch / Umeyama without scale)
    # H = Pred^T * GT
    H = pred_centered.T @ gt_centered
    U, S, Vt = np.linalg.svd(H)
    R_align = Vt.T @ U.T
    
    # Handle reflection
    if np.linalg.det(R_align) < 0:
        Vt[2, :] *= -1
        R_align = Vt.T @ U.T
        
    pred_aligned = pred_centered @ R_align.T
    
    # Calculate APE
    error = np.linalg.norm(gt_centered - pred_aligned, axis=1)
    rmse = np.sqrt(np.mean(error**2))
    
    print(f"APE RMSE: {rmse:.4f} m")
    
    # Plot
    plt.figure(figsize=(12, 6))
    
    plt.subplot(1, 2, 1)
    plt.plot(gt_centered[:, 0], gt_centered[:, 1], 'k-', label='Ground Truth')
    plt.plot(pred_aligned[:, 0], pred_aligned[:, 1], 'r--', label='FastLIO (Pred)')
    plt.title(f"Trajectory (Aligned), RMSE={rmse:.3f}m")
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.legend()
    plt.axis('equal')
    
    plt.subplot(1, 2, 2)
    plt.plot(error)
    plt.title("Absolute Position Error over Time")
    plt.xlabel("Frame")
    plt.ylabel("Error (m)")
    plt.grid(True)
    
    save_path = output_dir / "evaluation_result.png"
    plt.savefig(save_path)
    print(f"Plot saved to {save_path}")
    
    # Save text metrics
    with open(output_dir / "metrics.txt", "w") as f:
        f.write(f"RMSE: {rmse}\n")
        f.write(f"Mean Error: {np.mean(error)}\n")
        f.write(f"Max Error: {np.max(error)}\n")

if __name__ == "__main__":
    main()
