# ME5400: Advanced Robotics and Autonomous Systems

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![ROS Version](https://img.shields.io/badge/ROS-Noetic-blue.svg)](http://wiki.ros.org/noetic)
[![Python Version](https://img.shields.io/badge/Python-3.8+-green.svg)](https://www.python.org/)

Language / 语言: [中文](./README.md) | **English**

This repository contains the ME5400 course project. It focuses on an integrated **FAST-LIO odometry + MCTrack multi-object tracking pipeline** for autonomous driving, combining real-time LiDAR/IMU pose estimation with 3D detection results for unified processing and visualization.

## Contents

- [Project Overview](#project-overview)
- [Repository Layout](#repository-layout)
- [Environment and Installation](#environment-and-installation)
- [Data Preparation](#data-preparation)
- [Usage Guide](#usage-guide)
- [Agent Development Workflow](#agent-development-workflow)
- [Configuration Reference](#configuration-reference)

## Project Overview

This project implements a **bidirectional closed-loop fusion system between LiDAR-based SLAM and multi-object tracking (MOT)**. The core idea is to exploit the complementarity between odometry and tracking:

- FAST-LIO provides high-frequency, accurate ego poses to stabilize MCTrack.
- MCTrack feeds dynamic-object awareness back into FAST-LIO to suppress moving-object ghosting and improve map quality.

### Repository Layout

```text
ME5400/
├── catkin_ws/                         # ROS workspace
│   ├── src/fast_lio/                  # FAST-LIO2 source tree with project-specific configs
│   │   ├── config/                    # Sensor and mapping parameters, e.g. velodyne.yaml
│   │   ├── launch/                    # ROS launch files
│   │   └── src/                       # FAST-LIO core implementation
│   ├── src/ME5400/                    # ROS nodes for MCTrack integration, markers, and bridges
│   └── src/ME5400/msg/                # Custom Detection3D / Detection3DArray messages
├── MCTrack/                           # Multi-object tracking framework, configs, and evaluation
├── MMDET3D/                           # PointPillars codebase, configs, weights, and data
│   └── local/ros/nodes/               # ROS detection node (kitti_pointpillars_bag_node.py)
├── Scripts/                           # Unified launch scripts grouped by subsystem
│   ├── evaluation/                    # Trajectory evaluation utilities (ATE/RPE)
│   ├── pointpillars/                  # PointPillars launch scripts
│   ├── fastlio/                       # FAST-LIO launch scripts
│   ├── mctrack/                       # MCTrack launch scripts
│   ├── joint_backend/                 # Optional ego-only joint backend scripts
│   ├── offline/                       # Optional semi-synchronous offline feeder scripts
│   ├── rviz/                          # RViz config and helpers
│   ├── utils/                         # Common utilities, build helpers, converters, etc.
│   ├── run_all.sh                     # One-command online comparison pipeline
│   └── run_offline_all.sh             # One-command offline comparison pipeline
├── Data_Tracking/                     # KITTI Tracking data and sample rosbags
├── Results/                           # Metrics, plots, archived trajectories, and generated PCD maps
├── agent/                             # Long-running agent harness and handoff files
├── image/                             # Project figures and image assets
└── README.md
```

Architecture notes:

- `Scripts/` groups startup logic by subsystem (`pointpillars`, `fastlio`, `mctrack`, `rviz`, `utils`) so the pipeline stays maintainable.
- `MMDET3D/` is kept mainly for PointPillars-related configs, checkpoints, and data.
- `MCTrack/` contains the tracking framework itself.
- `catkin_ws/` is the ROS integration layer that ties FAST-LIO and MCTrack together.

## Environment and Installation

### Requirements

#### System

- Ubuntu 20.04 LTS
- ROS Noetic
- Python 3.8+

#### Basic dependencies

```bash
# ROS dependencies
sudo apt install ros-noetic-pcl-ros ros-noetic-eigen-conversions

# Python dependencies
pip install numpy opencv-python matplotlib
```

#### PointPillars quick environment setup (`ME5400`)

- **Dataset path**: `MMDET3D/data/kitti/`
  The sample rosbag is stored at `MMDET3D/data/kitti/seq_0019_with_det.bag`.
- **Conda environment**: `ME5400` with Python `3.10.x`
- **Key versions**:
  CUDA Toolkit `12.8` with NVCC, PyTorch `2.10.0.dev+cu128` nightly, MMCV `2.1.0` built against CUDA 12.8, MMEngine `0.10.7`, MMDetection `3.2.0`, and MMDetection3D `1.4.0`
- **Create and activate the environment**

  ```bash
  conda create -n ME5400 python=3.10
  conda activate ME5400
  ```

- **Install CUDA 12.8**

  1. Install an NVIDIA 550+ driver and download [CUDA Toolkit 12.8](https://developer.nvidia.com/cuda-12-8-0-download-archive).
  2. Install the `cuda-toolkit` component only if your driver is already present.
  3. Add the following to `~/.bashrc`:

     ```bash
     export CUDA_HOME=/usr/local/cuda-12.8
     export PATH=$CUDA_HOME/bin:$PATH
     export LD_LIBRARY_PATH=$CUDA_HOME/lib64:$LD_LIBRARY_PATH
     ```

     Then run:

     ```bash
     source ~/.bashrc
     ```

- **Install PyTorch nightly for CUDA 12.8**

  ```bash
  pip install --upgrade pip
  pip install --pre torch torchvision torchaudio --index-url https://download.pytorch.org/whl/nightly/cu128
  ```

- **Install OpenMMLab components**

  ```bash
  pip install openmim
  mim install mmengine==0.10.7
  export CUDA_HOME=${CUDA_HOME:-/usr/local/cuda-12.8}
  MMCV_WITH_OPS=1 FORCE_CUDA=1 MAX_JOBS=4 mim install mmcv==2.1.0
  pip install mmdet==3.2.0 mmdet3d==1.4.0
  ```

  `mmcv` compiles CUDA/C++ ops during installation and usually takes 5 to 10 minutes. If memory is limited, keep `MAX_JOBS=4` or reduce it further.

  `mmdet3d==1.4.0` currently requires `mmcv < 2.2.0`, so do not upgrade MMCV beyond that range.

- **Common runtime dependencies**

  ```bash
  pip install open3d==0.19.0 opencv-python==4.12.0.88 numpy==2.1.2 matplotlib==3.10.6 scipy==1.15.3 \
              scikit-learn==1.7.2 pandas==2.3.3 pillow==11.3.0 pyyaml==6.0.3 tqdm terminaltables==3.1.10 \
              shapely==1.8.5.post1 pyquaternion==0.9.9 trimesh==4.8.3 plyfile==1.1.2 imageio==2.37.0 \
              fire==0.7.1 tensorboard==2.20.0 protobuf==6.32.1
  pip install rospkg==1.6.0 catkin-pkg==1.1.0 pycryptodomex==3.23.0 python-gnupg==0.5.6 pykitti==0.3.1
  ```

  If you want to run `offline_bag_feeder.py`, the offline evaluation scripts, or other tools that depend on `rosbag` and KITTI utilities directly inside the `ME5400` conda environment, you must also install `pycryptodomex`, `python-gnupg`, and `pykitti`. Otherwise you will usually see import errors such as `No module named 'Cryptodome'`, `No module named 'gnupg'`, or `No module named 'pykitti'`.

- **Verify the installation**

  ```bash
  python -c "import torch; print('PyTorch version:', torch.__version__); print('CUDA available:', torch.cuda.is_available()); print('Compiled arch list:', torch.cuda.get_arch_list())"
  python -c "import mmcv; print('MMCV version:', mmcv.__version__)"
  python -c "import mmdet3d; print('MMDetection3D version:', mmdet3d.__version__)"
  python -c "import rosbag, gnupg, pykitti; from Cryptodome.Cipher import AES; print('ROS bag dependency check passed')"
  ```

### Installation steps

#### 1. Clone the repository

```bash
git clone https://github.com/XC-CN/ME5400.git
cd ME5400
```

#### 2. Build FAST-LIO2

```bash
cd catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

#### 3. Install MCTrack dependencies

```bash
cd MCTrack
pip install -r requirements.txt
```

## Data Preparation

This project depends on the [KITTI Tracking Benchmark](http://www.cvlibs.net/datasets/kitti/eval_tracking.php). Download and organize the dataset as follows.

### 1. Download the dataset

Download these files from the [KITTI Tracking Benchmark website](http://www.cvlibs.net/datasets/kitti/eval_tracking.php):

- **Velodyne point clouds (29 GB)**: `data_tracking_velodyne.zip`
- **Training labels of object data (5 MB)**: `data_tracking_label_2.zip`
- **Camera calibration matrices of object data (1 MB)**: `data_tracking_calib.zip`
- **GPS/IMU data**: `data_tracking_oxts.zip`

### 2. Arrange the directory structure

Extract the files into `Data_Tracking/` so the layout becomes:

```text
ME5400/
└── Data_Tracking/
    └── training/
        ├── calib/              # from data_tracking_calib.zip
        ├── label_02/           # from data_tracking_label_2.zip
        ├── oxts/               # from data_tracking_oxts.zip
        └── velodyne/           # from data_tracking_velodyne.zip
```

Notes:

- `MMDET3D/data/kitti` is the directory expected by MMDetection3D. In practice it is usually a symlink to `Data_Tracking/` or an equivalent custom path.

## Usage Guide

> The current detector is an MMDetection3D PointPillars inference node publishing to `/detection/bboxes_3d` and `/detection/kitti_tracking`. An additional `/detection/lidar_detections` topic (`Detection3DArray` with the original point cloud timestamp) is also available. If you replace the detector later, keep the topic interfaces consistent.
>
> In the default pipeline, RViz and FAST-LIO consume `/kitti/velo/pointcloud` and `/kitti/oxts/imu` directly.

Before running anything, prepare the KITTI Tracking data directories used for playback and MCTrack calibration:

- `Data_Tracking/training/` must contain subdirectories such as `calib/`, `oxts/`, and `velodyne/`.
- `Data_Tracking/det_tracking_lsvm/` is only needed when evaluating historical detections. It can be ignored for real-time detection.
- Convert LiDAR + IMU data into rosbags and place them under `Data_Tracking/rosbags/`, or use the sample bags already included in the repository.

> The current workflow is still driven by replaying KITTI Tracking rosbags and is not wired directly to live sensors yet.
>
> `kitti_tracking_to_rosbag.py` now auto-loads `Scripts/utils/setup_runtime_env.sh`, so you usually do not need to `source` the environment manually first.

You can generate the required rosbag from raw KITTI Tracking data with the provided helper:

```bash
./Scripts/utils/kitti_tracking_to_rosbag.py --seq 20
```

### Playable rosbags

| File | Scene | Path |
| :--- | :--- | :--- |
| **`seq_0019_nodet.bag`** | Urban street | `Data_Tracking/rosbags/seq_0019_nodet.bag` |
| **`seq_0020_nodet.bag`** | Highway | `Data_Tracking/rosbags/seq_0020_nodet.bag` |
| **`seq_0009_nodet.bag`** | Urban street | `Data_Tracking/rosbags/seq_0009_nodet.bag` |

### One-command startup (recommended)

If the environment and data are ready, use the automation scripts included in this repository. The project offers both online and offline shortcuts so each subsystem stays separated and easier to debug.

#### 1. Full automatic comparison run (recommended)

This is the most convenient entry point. The script first runs a clean baseline, then automatically cleans up the background processes and launches the optimized perception-closed-loop pipeline. In the online optimized stage it now also starts `joint_backend_ego`, and saves an additional joint-backend trajectory for `/joint_backend/odom`.

```bash
# Default: headless, no visualization, plots written under 0020_results/
./Scripts/run_all.sh 0020

# Recommended: enable RViz and render the runtime process on screen
./Scripts/run_all.sh --viz 0020

# Also export the FAST-LIO global map
./Scripts/run_all.sh --save-map 0020
```

By default the scripts now run headless unless `--viz` is specified.

#### 2. Run individual pipeline variants

If you only need one branch for debugging or fixed-data experiments, call the dedicated scripts directly. They are headless by default and can be switched to visualization mode with `--viz`.

```bash
# Clean baseline only: native FAST-LIO mapping, no pose_bridge, no perception chain
./Scripts/run_baseline.sh 0020
./Scripts/run_baseline.sh --viz 0020

# Optimized pipeline only: PointPillars + MCTrack + pose_bridge + dynamic-weight FAST-LIO + joint backend
./Scripts/run_optimized.sh 0020
./Scripts/run_optimized.sh --viz 0020
./Scripts/run_optimized.sh --save-map 0020
```

#### 3. Offline comparison run (semi-synchronous feeder + automatic evaluation)

```bash
./Scripts/run_offline_all.sh 0020
./Scripts/run_offline_all.sh --viz 0020
./Scripts/run_offline_all.sh --save-map 0020
```

What the script does:

- Runs `Baseline(Offline,NoWeight)` and `JointOffline` sequentially.
- In stage 1, the offline feeder drives FAST-LIO without waiting for detection/tracking outputs.
- In stage 2, PointPillars, MCTrack, FAST-LIO, and the joint backend all run together while the offline feeder advances the bag semi-synchronously.
- Automatically generates:
  - `Results/<SEQ>_results/trajectory_baseline_offline.txt`
  - `Results/<SEQ>_results/trajectory_weighted_offline.txt`
  - `Results/<SEQ>_results/trajectory_joint.txt`
  - `Results/<SEQ>_results/metrics.txt`

All baseline and optimized trajectories (`trajectory*.txt`), plots (`.png`), and summary metrics (`metrics.txt` / `metrics.json`) are archived automatically under `Results/<SEQ>_results/`. Point-cloud maps (`.pcd`) are generated and archived only when `--save-map` is passed explicitly.

#### 4. Clean up leftover ROS nodes and processes

If the previous run crashed, or if manual step-by-step debugging left stale ROS nodes, `rosbag play`, RViz, FAST-LIO, PointPillars, or MCTrack processes behind, run:

```bash
./Scripts/utils/cleanup_ros_runtime.sh
```

Behavior of the cleanup script:

- Sends `SIGTERM` first, then escalates to `SIGKILL` one second later if a target process is still alive.
- Cleans `rviz`, `publish_gt_path.py`, `rosbag play`, `offline_bag_feeder.py`, FAST-LIO processes, `kitti_pointpillars_bag_node.py`, `mctrack_online_node.py`, `joint_backend_ego_node.py`, and `odom_to_tum_recorder.py`.
- If a usable `roscore` is detected, also runs `rosnode cleanup` to remove stale registrations from the ROS master.

Recommendations:

- Run it once before manual step-by-step startup to avoid stale topics or master registrations from interfering.
- `./Scripts/run_baseline.sh`, `./Scripts/run_optimized.sh`, and `./Scripts/run_offline_all.sh` already call it automatically on startup, so you usually do not need to run it manually.
- Do not run it if you still have other ROS jobs active that you do not want to terminate.

---

### Step-by-step startup guide

#### 1. Recommended first step: clear stale ROS runtime state

```bash
./Scripts/utils/cleanup_ros_runtime.sh
```

#### 2. Start `roscore` (Terminal A)

```bash
roscore
```

#### 3. Build the workspace after the first setup or after code changes

```bash
./Scripts/utils/build_catkin_ws.sh
```

#### 4. Optionally generate or refresh the default rosbag

```bash
./Scripts/utils/kitti_tracking_to_rosbag.py --seq 20
```

#### 5. Start the PointPillars detection node (Terminal B)

```bash
./Scripts/pointpillars/run_pointpillars_node.sh
```

#### 6. Start FAST-LIO (Terminal C)

FAST-LIO supports three typical launch modes:

```bash
# Dynamic-weight pipeline with pose_bridge enabled
./Scripts/fastlio/run_fastlio.sh both

# Clean baseline: native FAST-LIO mapping only
./Scripts/fastlio/run_fastlio.sh mapping use_dynamic_weights:=false

# Also save the global map on exit
./Scripts/fastlio/run_fastlio.sh both pcd_save_en:=true
```

Notes:

- `mapping` starts pure FAST-LIO and launches `laserMapping` only.
- `both` launches both mapping and `pose_bridge`, allowing the optimized pipeline and MCTrack to subscribe to FAST-LIO poses.
- Dynamic weighting is enabled by default and can be controlled via `use_dynamic_weights`.
- FAST-LIO publishes both `/Odometry` and `/Odometry_imu_predicted`. The optimized chain currently forwards `/Odometry -> /mctrack/lidar_pose` through `fastlio_pose_bridge`.

#### 7. Start the online MCTrack node (Terminal D)

```bash
./Scripts/mctrack/run_mctrack_online_node.sh
```

#### 8. Drive the data source (Terminal F, choose one)

Online mode:

```bash
rosparam set use_sim_time true

# Highway scene
rosbag play Data_Tracking/rosbags/seq_0020_nodet.bag --clock

# Urban pedestrian-dense scene
rosbag play Data_Tracking/rosbags/seq_0019_nodet.bag --clock
```

Use `--loop` if you want continuous replay.

When stopping the system, shut down the FAST-LIO mapping process before stopping `rosbag`.

Offline semi-synchronous mode:

```bash
./Scripts/offline/run_offline_feeder.sh \
  --bag Data_Tracking/rosbags/seq_0020_nodet.bag
```

The offline feeder reads the bag frame by frame. After publishing each LiDAR frame, it waits for detection and tracking results, subject to configurable timeouts, before advancing to the next frame. This mode is intended to avoid frame loss when PointPillars cannot keep up with real-time playback. It does not replace or modify the original online mode.

For a full offline comparison run, prefer:

```bash
./Scripts/run_offline_all.sh --headless 0020
```

#### 9. Open RViz (Terminal E, together with rosbag playback)

```bash
./Scripts/rviz/run_rviz.sh
```

This uses `Scripts/rviz/ME5400.rviz` to visualize point clouds, PointPillars detections, and MCTrack results.

By default, the script tries to launch RViz maximized on the non-primary monitor. On the current machine that corresponds to `HDMI-0` / `SKYDATA`. To override the target screen, use one of the following:

```bash
RVIZ_TARGET_MONITOR=secondary ./Scripts/rviz/run_rviz.sh
RVIZ_TARGET_MONITOR=HDMI-0 ./Scripts/rviz/run_rviz.sh
RVIZ_TARGET_MONITOR=1 ./Scripts/rviz/run_rviz.sh
RVIZ_TARGET_MONITOR=other ./Scripts/rviz/run_rviz.sh
```

#### 10. Optionally start the ego-only joint backend (Terminal G)

This module does not replace FAST-LIO. It publishes an additional optimized ego odometry stream on `/joint_backend/odom`.

```bash
./Scripts/joint_backend/run_joint_backend_ego.sh
```

You can pass a custom config file:

```bash
./Scripts/joint_backend/run_joint_backend_ego.sh \
  --config ~/ME5400/catkin_ws/src/ME5400/config/joint_backend_ego.yaml
```

## Agent Development Workflow

To make long-running agent handoffs stable, this repository includes a minimal harness:

- `agent/init.sh`: checks repository paths, ROS environment, conda environment, and dataset locations, and can optionally trigger `catkin_make`
- `agent/check_smoke.sh`: runs a quick smoke test on existing trajectories by default; use `--full` to run `run_all.sh --headless`
- `agent/feature_list.json`: machine-readable acceptance items; an agent should only advance one `passes=false` item at a time
- `agent/progress.md`: append-only handoff log recording changes, validation results, risks, and next steps

Recommended session order:

```bash
# 1) Rebuild context
bash agent/init.sh --check-only --seq 0020

# 2) Read handoff and acceptance status
cat agent/progress.md
cat agent/feature_list.json
git log --oneline -5

# 3) Run a quick validation pass before changing code
bash agent/check_smoke.sh --seq 0020
```

For a real end-to-end regression run:

```bash
bash agent/check_smoke.sh --full --seq 0020
```

Notes:

- `Results/<SEQ>_results/metrics.txt` is kept unchanged for compatibility with the README and manual inspection.
- The evaluation pipeline now also emits `Results/<SEQ>_results/metrics.json` for machine-readable agent checks.
- The default quick smoke path only re-runs evaluation and does not replay the full bag, making it a low-cost baseline before and after each change.

## Configuration Reference

Main FAST-LIO config: `catkin_ws/src/fast_lio/config/velodyne.yaml`

```yaml
common:
  lid_topic: "/kitti/velo/pointcloud"  # Raw LiDAR topic
  imu_topic: "/kitti/oxts/imu"         # Raw IMU topic

preprocess:
  lidar_type: 2                        # Velodyne LiDAR
  scan_line: 64                        # Number of scan lines
  scan_rate: 10                        # Scan rate
  ignore_given_offset_time: true       # Ignore estimated time field for the current KITTI bags

mapping:
  extrinsic_T: [0, 0, 0.28]           # Translation extrinsic
  extrinsic_R: [1, 0, 0,              # Rotation extrinsic
                0, 1, 0,
                0, 0, 1]
```

### FAST-LIO dynamic-object down-weighting

What it does:

- FAST-LIO can down-weight dynamic objects to reduce the impact of moving vehicles on mapping.
- The feature is controlled by `use_dynamic_weights` in `mapping_velodyne.launch`, and is enabled by default.
- The node subscribes to `/mctrack/tracked_objects` and computes a weight for each tracked object based on fused velocity, acceleration, and detection confidence.

How it works:

- **Weight injection**: during LiDAR preprocessing, each point is checked against dynamic-object bounding boxes. If a point falls inside a box, the computed dynamic weight is combined with the original `intensity` by taking the minimum. High-intensity points can therefore be down-weighted, while already-low points stay unchanged.
- **Optimization down-weighting**: in backend optimization, FAST-LIO reads that `intensity` value and uses it to scale both the Jacobian and the residual, reducing the influence of fast-moving, high-confidence objects.
- **Temporal loop**:
  - **Time T**: FAST-LIO estimates pose `P_t` from IMU preintegration and the current point cloud, using dynamic weights returned from the previous frame (`T-1`).
  - **Time T**: MCTrack receives `P_t` and the point cloud, performs detection and tracking, and outputs a dynamic-object list `O_t`.
  - **Time T+1**: FAST-LIO receives `O_t` and uses it to weight the next point cloud.

Usage:

```bash
# Enable dynamic weighting and pose_bridge
./Scripts/fastlio/run_fastlio.sh both use_dynamic_weights:=true

# Clean baseline: native FAST-LIO mapping only
./Scripts/fastlio/run_fastlio.sh mapping use_dynamic_weights:=false
```

Parameter notes:

- Related knobs such as penalty coefficients, velocity and acceleration thresholds, bbox clipping bounds, and timeout thresholds are exposed as `preprocess/dynamic_*`.
- See `catkin_ws/src/fast_lio/launch/mapping_velodyne.launch` for the full list, and override them directly on the launch command when needed.

### FAST-LIO map saving

Where maps are stored:

- The automation scripts (`run_all.sh`, `run_offline_all.sh`, `run_baseline.sh`, and `run_optimized.sh`) do **not** save FAST-LIO maps by default.
- A map is archived only when `--save-map` is passed explicitly.
- Archived outputs are placed under `Results/<SEQ>_results/scans_*.pcd`.
- In the default configuration, all scan frames accumulate into a single PCD file.

How to enable it:

```bash
./Scripts/run_baseline.sh --save-map 0020
./Scripts/run_optimized.sh --save-map 0020
./Scripts/run_all.sh --save-map 0020
./Scripts/run_offline_all.sh --save-map 0020

./Scripts/fastlio/run_fastlio.sh both pcd_save_en:=true
```

Save timing:

- The map is written automatically when the FAST-LIO process exits cleanly.
- In `both` mode, the wrapper script shuts the process down gracefully with `SIGTERM` and waits up to 45 seconds for map saving to finish.
- If the process still does not exit after 45 seconds, the script force-kills it with `SIGKILL`.

Progress reporting:

- FAST-LIO prints detailed save progress information, including:
  - point count
  - estimated file size
  - output path
  - actual file size and elapsed time
  - save success or failure

Archival strategy:

- The automation scripts archive only a newly generated `scans.pcd` from the current run.
- Leftover temporary files from older runs are ignored and will not be archived into the current result directory.

Recommended interruption method:

```bash
# Method 1: press Ctrl+C on the wrapper script
# The script will handle graceful shutdown, wait for map saving, and verify the output

# Method 2: when mapping is launched manually, press Ctrl+C there
# FAST-LIO catches the signal, exits cleanly, and saves the map with progress output
```

Important notes:

- To avoid wasting disk space and startup/shutdown time, `pcd_save/pcd_save_en` in `velodyne.yaml` is now disabled by default.
- If you do need a global `.pcd` map for offline visualization, prefer `--save-map` on the automation scripts or pass `pcd_save_en:=true` directly.
- The script waits up to 45 seconds for map saving to complete.
- If the process is killed forcefully with `kill -9`, the map may not be saved.
- The output directory can be overridden via `pcd_save/output_dir`.
- If the point cloud is empty, no map file will be generated, which is expected behavior.

### Joint Backend (ego-only) configuration

Config file: `catkin_ws/src/ME5400/config/joint_backend_ego.yaml`

- `window_size`: sliding-window length in frames
- `lambda_obj`: global weight for object-observation constraints
- `score_th`: detection confidence threshold
- `track_len_th`: minimum track length threshold
- `max_dt`: maximum allowed timestamp gap between object messages and odometry
- `max_dv`: velocity jump threshold used to reject unstable objects
- `odom_noise_fallback`: fallback noise `[sigma_x, sigma_y, sigma_yaw]` used when `/Odometry` covariance is abnormal

### Joint Backend (ego-only) tuning suggestions

Recommended tuning order:

1. **`lambda_obj`** is the most important knob because it controls how strongly object observations constrain the solution. A practical range is `0.1` to `0.6`, usually starting from `0.3`.
   - If the trajectory is pulled too hard by tracked objects or becomes jittery, reduce it.
   - If the optimization barely changes the result, increase it.
2. **`window_size`** is typically best in the `8` to `15` range, with `10` as the default.
   - Smaller: faster response, noisier estimate
   - Larger: smoother estimate, more lag
3. **Gate parameters: `score_th`, `track_len_th`, `max_dt`, `max_dv`**
   - `score_th`: typically `0.5` to `0.6`; higher is more conservative
   - `track_len_th`: typically `3` to `5`; higher is more stable
   - `max_dt`: typically `0.2` to `0.3 s`; too small drops constraints, too large risks mismatches
   - `max_dv`: typically `2.0` to `3.0`; smaller values reject velocity spikes more aggressively
4. **`min_obj_weight`** sets the lower bound of object constraints. A good range is `0.1` to `0.2`.
5. **`odom_noise_fallback`** only applies when `/Odometry` covariance is invalid. The default `[0.20, 0.20, 0.10]` is usually a good choice.

Suggested experiment loop:

1. Keep the same rosbag and playback rate fixed.
2. Tune the joint backend first with `use_dynamic_weights:=false`.
3. Then switch to `use_dynamic_weights:=true` and tune jointly, usually with a slightly smaller `lambda_obj`.

### Offline Feeder (semi-synchronous offline scheduling) configuration

Config file: `catkin_ws/src/ME5400/config/offline_bag_feeder.yaml`

- `bag_path`: rosbag path to read offline
- `lidar_topic`: LiDAR topic used as the master clock that advances the feeder frame by frame
- `det_topic` / `track_topic`: key output topics that the feeder waits for on each frame
- `require_detection` / `require_tracking`: whether those outputs are mandatory before advancing
- `stamp_tolerance`: timestamp matching window between a key output and the current frame
- `timeout_det` / `timeout_track`: waiting timeouts for detection and tracking
- `publish_clock`: whether to publish `/clock`, recommended to keep enabled
- `clock_step_sec`: simulation clock step used while waiting for outputs
