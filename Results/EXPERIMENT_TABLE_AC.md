# ME5400 Experiment Master Table (Two-Track: A vs C)

## Comparison Setup (Fixed)
- A: `Baseline (Offline + Dynamic Weight OFF)`
- C: `JointOffline (Offline + Dynamic Weight ON + Joint Backend)`

> Current report uses two-track comparison only (WeightedOffline excluded).

## Current Result for Sequence 0020
- Ground Truth: `Data_Tracking/training/data_tracking_oxts/training/oxts/0020.txt`
- Evaluation output directory: `Results/0020_results/`
- Comparison plots:
  - `Results/0020_results/evaluation_joint_backend.png`
  - `Results/0020_results/ablation_bar.png`
- ATE metrics file:
  - `Results/0020_results/metrics_joint_backend.txt`
- KITTI Tr/Rot metrics file:
  - `Results/0020_results/metrics_kitti_tr_rot_ac.txt`

## ATE Table (Absolute Error)
| Seq | Baseline ATE RMSE (m) | Joint ATE RMSE (m) | RMSE Gain | Baseline ATE Mean (m) | Joint ATE Mean (m) | Mean Gain | Baseline ATE Max (m) | Joint ATE Max (m) | Max Gain | Baseline Frames | Joint Frames |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0001 | 2.3343 | 1.7317 | 25.82% | 2.1035 | 1.5658 | 25.56% | 4.1735 | 2.9875 | 28.42% | 426 | 428 |
| 0007 | 3.3231 | 2.6958 | 18.88% | 3.2743 | 2.4254 | 25.93% | 4.3839 | 13.8171 | -215.19% | 783 | 784 |
| 0009 | 2.4151 | 2.0039 | 17.03% | 2.3277 | 1.9321 | 17.00% | 3.5643 | 3.6356 | -2.00% | 787 | 787 |
| 0011 | 0.4965 | 1.0479 | -111.07% | 0.4125 | 0.8060 | -95.39% | 1.7097 | 3.3029 | -93.19% | 355 | 357 |
| 0020 | 29.1782 | 11.5856 | 60.29% | 26.3322 | 10.0928 | 61.67% | 39.9080 | 17.4796 | 56.20% | 820 | 821 |

## KITTI Relative Error Table (Tr./Rot.)
Setup: segment lengths `100,200,300,400,500,600,700,800 m`, start step `10` frames, timestamp tolerance `0.2 s`.

| Seq | Baseline Tr. (%) | Joint Tr. (%) | Tr. Gain | Baseline Rot. (deg/100m) | Joint Rot. (deg/100m) | Rot. Gain | Baseline Matched | Joint Matched | Baseline Segments | Joint Segments |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0001 | 4.4355 | 3.3913 | 23.54% | 2.0336 | 1.5419 | 24.18% | 426 | 428 | 39 | 39 |
| 0007 | 2.5718 | 2.8668 | -11.47% | 1.8037 | 2.3480 | -30.18% | 783 | 784 | 150 | 151 |
| 0009 | 2.6519 | 3.1584 | -19.10% | 1.3420 | 1.5859 | -18.18% | 787 | 787 | 258 | 258 |
| 0011 | 1.2282 | 1.5713 | -27.93% | 0.6825 | 0.7314 | -7.17% | 355 | 357 | 16 | 16 |
| 0020 | 6.0245 | 2.9930 | 50.32% | 0.6357 | 0.5312 | 16.44% | 820 | 821 | 282 | 282 |

## Multi-Sequence ATE Summary (To Be Filled)

| Seq | Baseline ATE RMSE (m) | Joint ATE RMSE (m) | RMSE Gain | Baseline ATE Mean (m) | Joint ATE Mean (m) | Mean Gain | Baseline ATE Max (m) | Joint ATE Max (m) | Max Gain | Notes |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|
| 0000 |  |  |  |  |  |  |  |  |  |  |
| 0001 | 2.3343 | 1.7317 | 25.82% | 2.1035 | 1.5658 | 25.56% | 4.1735 | 2.9875 | 28.42% | Done |
| 0002 |  |  |  |  |  |  |  |  |  |  |
| 0003 |  |  |  |  |  |  |  |  |  |  |
| 0004 |  |  |  |  |  |  |  |  |  |  |
| 0005 |  |  |  |  |  |  |  |  |  |  |
| 0006 |  |  |  |  |  |  |  |  |  |  |
| 0007 | 3.3231 | 2.6958 | 18.88% | 3.2743 | 2.4254 | 25.93% | 4.3839 | 13.8171 | -215.19% | Done |
| 0008 |  |  |  |  |  |  |  |  |  |  |
| 0009 | 2.4151 | 2.0039 | 17.03% | 2.3277 | 1.9321 | 17.00% | 3.5643 | 3.6356 | -2.00% | Done |
| 0010 |  |  |  |  |  |  |  |  |  |  |
| 0011 | 0.4965 | 1.0479 | -111.07% | 0.4125 | 0.8060 | -95.39% | 1.7097 | 3.3029 | -93.19% | Done |
| 0012 |  |  |  |  |  |  |  |  |  |  |
| 0013 |  |  |  |  |  |  |  |  |  |  |
| 0014 |  |  |  |  |  |  |  |  |  |  |
| 0015 |  |  |  |  |  |  |  |  |  |  |
| 0016 |  |  |  |  |  |  |  |  |  |  |
| 0017 |  |  |  |  |  |  |  |  |  |  |
| 0018 |  |  |  |  |  |  |  |  |  |  |
| 0019 |  |  |  |  |  |  |  |  |  |  |
| 0020 | 29.1782 | 11.5856 | 60.29% | 26.3322 | 10.0928 | 61.67% | 39.9080 | 17.4796 | 56.20% | Done |

## Multi-Sequence Tr./Rot. Summary (To Be Filled)

| Seq | Baseline Tr. (%) | Joint Tr. (%) | Tr. Gain | Baseline Rot. (deg/100m) | Joint Rot. (deg/100m) | Rot. Gain | Notes |
|---|---:|---:|---:|---:|---:|---:|---|
| 0000 |  |  |  |  |  |  |  |
| 0001 | 4.4355 | 3.3913 | 23.54% | 2.0336 | 1.5419 | 24.18% | Done |
| 0002 |  |  |  |  |  |  |  |
| 0003 |  |  |  |  |  |  |  |
| 0004 |  |  |  |  |  |  |  |
| 0005 |  |  |  |  |  |  |  |
| 0006 |  |  |  |  |  |  |  |
| 0007 | 2.5718 | 2.8668 | -11.47% | 1.8037 | 2.3480 | -30.18% | Done |
| 0008 |  |  |  |  |  |  |  |
| 0009 | 2.6519 | 3.1584 | -19.10% | 1.3420 | 1.5859 | -18.18% | Done |
| 0010 |  |  |  |  |  |  |  |
| 0011 | 1.2282 | 1.5713 | -27.93% | 0.6825 | 0.7314 | -7.17% | Done |
| 0012 |  |  |  |  |  |  |  |
| 0013 |  |  |  |  |  |  |  |
| 0014 |  |  |  |  |  |  |  |
| 0015 |  |  |  |  |  |  |  |
| 0016 |  |  |  |  |  |  |  |
| 0017 |  |  |  |  |  |  |  |
| 0018 |  |  |  |  |  |  |  |
| 0019 |  |  |  |  |  |  |  |
| 0020 | 6.0245 | 2.9930 | 50.32% | 0.6357 | 0.5312 | 16.44% | Done |

## Two-Track Evaluation Commands (A + C)

```bash
python3 Scripts/evaluation/evaluate_joint_backend.py \
  --gt Data_Tracking/training/data_tracking_oxts/training/oxts/0020.txt \
  --traj "Baseline(Offline,NoWeight):Results/0020_results/trajectory_baseline_offline.txt" \
  --traj "JointOffline:Results/0020_results/trajectory_joint.txt" \
  --output Results
```

```bash
python3 Scripts/evaluation/evaluate_kitti_tr_rot.py \
  --oxts Data_Tracking/training/data_tracking_oxts/training/oxts/0020.txt \
  --calib Data_Tracking/training/data_tracking_calib/training/calib/0020.txt \
  --traj "Baseline(Offline,NoWeight):Results/0020_results/trajectory_baseline_offline.txt" \
  --traj "JointOffline:Results/0020_results/trajectory_joint.txt" \
  --output Results/0020_results/metrics_kitti_tr_rot_ac.txt
```
