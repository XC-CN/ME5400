#!/bin/bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PROJECT_ROOT=$(cd "$SCRIPT_DIR/../.." && pwd)

source "$PROJECT_ROOT/Scripts/utils/setup_ros_env.sh"

bash "$PROJECT_ROOT/Scripts/utils/cleanup_ros_runtime.sh" >/tmp/jitter_cleanup.log 2>&1 || true

if ! rostopic list >/dev/null 2>&1; then
  roscore >/tmp/jitter_roscore.log 2>&1 &
  sleep 5
fi

cleanup_all() {
  bash "$PROJECT_ROOT/Scripts/utils/cleanup_ros_runtime.sh" >/tmp/jitter_cleanup_post.log 2>&1 || true
  pkill -f trace_detection_tracking_jitter.py 2>/dev/null || true
  pkill -f "rosbag play $PROJECT_ROOT/Data_Tracking/rosbags/seq_0020_nodet.bag" 2>/dev/null || true
}
trap cleanup_all EXIT INT TERM

BAG_FILE="" bash "$PROJECT_ROOT/Scripts/pointpillars/run_pointpillars_node.sh" _seq:=0020 >/tmp/pp_jitter.log 2>&1 &
bash "$PROJECT_ROOT/Scripts/mctrack/run_mctrack_online_node.sh" >/tmp/mctrack_jitter.log 2>&1 &
bash "$PROJECT_ROOT/Scripts/fastlio/run_fastlio.sh" both use_dynamic_weights:=true lidar_topic:=/kitti/velo/pointcloud imu_topic:=/kitti/oxts/imu >/tmp/fastlio_jitter.log 2>&1 &

sleep 18
source "$PROJECT_ROOT/Scripts/utils/setup_ros_env.sh"
rosnode list
rosbag play "$PROJECT_ROOT/Data_Tracking/rosbags/seq_0020_nodet.bag" --clock --duration=20 --topics /kitti/velo/pointcloud /kitti/oxts/imu >/tmp/bag_jitter.log 2>&1 &
python3 "$PROJECT_ROOT/Scripts/utils/trace_detection_tracking_jitter.py" --duration 15 --output-prefix /tmp/me5400_jitter/run2 >/tmp/jitter_trace.log 2>&1
python3 - <<'PY'
import csv
import math
import os

base = "/tmp/me5400_jitter/run2"
for name in ["detections", "pp_markers", "track_markers"]:
    path = f"{base}_{name}.csv"
    print(f"FILE {path} exists={os.path.exists(path)}")


def read_rows(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def vals(rows, key):
    out = []
    for row in rows:
        value = row[key]
        if value in ("", "nan", "NaN"):
            continue
        try:
            parsed = float(value)
        except ValueError:
            continue
        if math.isfinite(parsed):
            out.append(parsed)
    return out


det = read_rows(base + "_detections.csv")
pp = read_rows(base + "_pp_markers.csv")
trk = read_rows(base + "_track_markers.csv")
print("DETECTIONS", len(det))
print("PP_MARKERS", len(pp))
print("TRACK_MARKERS", len(trk))
for label, rows, key in [
    ("det_publish_lag", det, "publish_lag_sec"),
    ("det_latest_pose_minus_det", det, "latest_pose_minus_det_sec"),
    ("det_nearest_pose_minus_det", det, "nearest_pose_minus_det_sec"),
    ("det_latest_vs_nearest_gap_m", det, "latest_vs_nearest_pose_gap_m"),
    ("pp_publish_lag", pp, "publish_lag_sec"),
    ("track_publish_lag", trk, "publish_lag_sec"),
]:
    xs = vals(rows, key)
    if not xs:
        print(label, "EMPTY")
        continue
    abs_x = [abs(x) for x in xs]
    print(
        label,
        "mean=",
        round(sum(xs) / len(xs), 4),
        "mean_abs=",
        round(sum(abs_x) / len(abs_x), 4),
        "max_abs=",
        round(max(abs_x), 4),
    )
locked = [row["frame_locked"] for row in pp if row.get("frame_locked") not in ("", None)]
if locked:
    ones = sum(1 for value in locked if value == "1")
    print("pp_frame_locked", ones, "/", len(locked))
PY

printf '\n---- tracer log ----\n'
tail -n 20 /tmp/jitter_trace.log || true
printf '\n---- pointpillars tail ----\n'
tail -n 40 /tmp/pp_jitter.log || true
printf '\n---- mctrack tail ----\n'
tail -n 40 /tmp/mctrack_jitter.log || true
printf '\n---- fastlio tail ----\n'
tail -n 60 /tmp/fastlio_jitter.log || true
