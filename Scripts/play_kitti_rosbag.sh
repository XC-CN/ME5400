#!/bin/bash
set -e

ROOT_DIR=$(cd "$(dirname "$0")/.." && pwd)
BAG_PATH="$ROOT_DIR/tracking/rosbags/seq_0019_with_det.bag"

if [[ ! -f "$BAG_PATH" ]]; then
  echo "[错误] bag 文件不存在: $BAG_PATH" >&2
  exit 1
fi

rosparam set use_sim_time true
rosbag play "$BAG_PATH" --clock --loop
