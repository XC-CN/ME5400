# Agent Progress

This file is append-only. Each session should record:
- what changed
- what was verified
- what remains risky or blocked
- the next recommended step

## 2026-04-06

Summary:
- Added a minimal long-running agent harness under `agent/`.
- Added `agent/init.sh` for fresh-session environment checks.
- Added `agent/check_smoke.sh` with quick and full validation modes.
- Added `agent/feature_list.json` as a machine-readable acceptance list.
- Extended `Scripts/evaluation/evaluate_trajectory.py` to emit `metrics.json` while keeping `metrics.txt` unchanged for README and manual inspection.
- Fixed `Scripts/utils/setup_ros_env.sh` so it resolves `catkin_ws` from the project root.

Verified:
- `bash agent/check_smoke.sh --seq 0020` passed.
- The initializer path, quick smoke path, and machine-readable metrics generation were verified on 2026-04-06.

Open risks:
- Full headless pipeline validation for `0019` and `0020` has not been re-run in this session yet.
- Existing `run_baseline.sh` and `run_optimized.sh` still use broad startup cleanup and should be narrowed in a later hardening pass.

Next:
1. When compute is available, run `bash agent/check_smoke.sh --full --seq 0020`.
2. If the full 0020 run passes, mark the corresponding integration item in `agent/feature_list.json`.
3. Repeat the full validation for `0019`.

## 2026-04-06 (PointPillars profiling)

Summary:
- Added stage-level wall-clock timing to [`MMDET3D/local/ros/nodes/kitti_pointpillars_bag_node.py`](/home/xc/Projects/ME5400/MMDET3D/local/ros/nodes/kitti_pointpillars_bag_node.py) without changing the existing PointPillars execution path.
- Timing now splits `pointcloud_to_numpy`, temp `.bin` write/remove, `inferencer_call`, result extraction, camera projection, ROS message publishing, compute-only total, and rate-control sleep.
- Replaced the old `rospy.Time.now()` processing timer with `time.perf_counter()` so bag `/clock` and deliberate publish-rate sleep no longer distort performance diagnosis.

Verified:
- `python3 -m py_compile MMDET3D/local/ros/nodes/kitti_pointpillars_bag_node.py`
- `bash agent/check_smoke.sh --seq 0020`

Open risks:
- Quick smoke does not execute the PointPillars ROS node, so the new timing logs have not yet been observed in a live bag run.
- The current profiling preserves the existing slow path (PointCloud2 Python loop + temp `.bin` write + `LidarDet3DInferencer` high-level call); follow-up optimization should be based on the first real timing output.

Next:
1. Run the PointPillars node on a short bag segment and inspect `PointPillars计时汇总(...)` plus `慢帧 ... 分段=...` logs.
2. If `inferencer_call` dominates, replace the temp-file path with ndarray input or a lower-overhead model API.
3. If `pointcloud_to_numpy` or publish stages dominate, optimize ROS conversion/publication separately instead of touching the model first.

## 2026-04-06 (PointPillars profiling results)

Summary:
- Ran a short live profiling session for the PointPillars ROS node and captured the timing log at [`agent/logs/pointpillars_profile_seq0020_20260406_214054.log`](/home/xc/Projects/ME5400/agent/logs/pointpillars_profile_seq0020_20260406_214054.log).
- Confirmed that the dominant bottleneck is `pointcloud_to_numpy`, not the model forward itself.
- The temp `.bin` write/remove path is measurable but negligible compared with point cloud parsing.

Verified:
- A real PointPillars run on seq `0020` produced repeated timing summaries and slow-frame breakdowns in [`agent/logs/pointpillars_profile_seq0020_20260406_214054.log`](/home/xc/Projects/ME5400/agent/logs/pointpillars_profile_seq0020_20260406_214054.log).
- Representative windows:
  - `PointPillars计时汇总(20帧均值)` at [`agent/logs/pointpillars_profile_seq0020_20260406_214054.log#L106`](/home/xc/Projects/ME5400/agent/logs/pointpillars_profile_seq0020_20260406_214054.log#L106): `点云解析=102.8ms`, `模型推理=44.3ms`, `纯计算总耗时=150.1ms`
  - `PointPillars计时汇总(20帧均值)` at [`agent/logs/pointpillars_profile_seq0020_20260406_214054.log#L650`](/home/xc/Projects/ME5400/agent/logs/pointpillars_profile_seq0020_20260406_214054.log#L650): `点云解析=95.5ms`, `模型推理=27.3ms`, `纯计算总耗时=125.6ms`
  - `慢帧 156` at [`agent/logs/pointpillars_profile_seq0020_20260406_214054.log#L637`](/home/xc/Projects/ME5400/agent/logs/pointpillars_profile_seq0020_20260406_214054.log#L637): `pointcloud_to_numpy=101.8ms`, `inferencer_call=24.2ms`
  - `慢帧 172` at [`agent/logs/pointpillars_profile_seq0020_20260406_214054.log#L700`](/home/xc/Projects/ME5400/agent/logs/pointpillars_profile_seq0020_20260406_214054.log#L700): occasional inference jitter with `inferencer_call=73.5ms`

Open risks:
- The current node still uses a Python loop in `_pointcloud2_to_numpy`, so the measured bottleneck is expected to persist until that path is rewritten.
- Inference occasionally spikes, but the average bottleneck is still point cloud conversion by a wide margin.

Next:
1. Optimize [`MMDET3D/local/ros/nodes/kitti_pointpillars_bag_node.py`](/home/xc/Projects/ME5400/MMDET3D/local/ros/nodes/kitti_pointpillars_bag_node.py) `_pointcloud2_to_numpy` first.
2. Re-run the same short profiling workflow and compare `pointcloud_to_numpy` and `纯计算总耗时` against the saved baseline log above.
3. Only after that decide whether replacing the temp `.bin` path or bypassing `LidarDet3DInferencer` is still worth doing.
