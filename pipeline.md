# 在线 ROS 联动融合管线（FAST-LIO × PointPillars × MCTrack）

> 目的：将 **点云/IMU 里程计（FAST-LIO）** 与 **3D 检测（PointPillars）** 对齐，驱动 **MCTrack 在线跟踪** 并在 RViz 中可视化输出。

```mermaid
flowchart TD
  %% 数据源
  subgraph S1[数据源 / 播放器]
    PC[/点云\n/kitti/velo/pointcloud/]
    IMU[/IMU\n/kitti/oxts/imu/]
    BAG[rosbag play\ntracking/rosbags/seq_0019_with_det.bag]
  end

  %% 3D检测
  subgraph S2[3D 检测（MMDetection3D）]
    PP[PointPillars ROS 节点\nMMDET3D/local/ros/nodes/kitti_pointpillars_bag_node.py]
    DETARR[/目标检测\n/kitti/detections\nDetection3DArray/]
    DETSTR[/检测字符串（可选）\n/detection/kitti_tracking\nString/]
  end

  %% 里程计
  subgraph S3[里程计（FAST-LIO2）]
    FL[FAST-LIO mapping\nroslaunch fast_lio mapping_velodyne.launch]
    ODOM[/里程计\n/Odometry\nnav_msgs/Odometry/]
  end

  %% 位姿桥接
  subgraph S4[位姿桥接（kitti_tracklets_viz）]
    BR[fastlio_pose_bridge.py]
    POSE[/位姿\n/mctrack/lidar_pose\ngeometry_msgs/PoseStamped/]
    TF[[TF\nmap → velodyne]]
  end

  %% 跟踪与可视化
  subgraph S5[跟踪与可视化（MCTrack Online）]
    MCT[mctrack_online_node.py\nBase3DTracker.track_single_frame]
    MK[/可视化 Marker\n/mctrack/markers\nMarkerArray/]
    RVIZ[RViz]
  end

  %% 数据流
  BAG --> PC
  BAG --> IMU

  PC --> PP
  PC --> FL
  IMU --> FL

  PP --> DETARR --> MCT
  PP --> DETSTR

  FL --> ODOM --> BR --> POSE --> MCT
  BR --> TF

  MCT --> MK --> RVIZ
```

## 启动脚本对照（与仓库脚本一致）

- PointPillars：
  - 直接启动：`python MMDET3D/local/ros/nodes/kitti_pointpillars_bag_node.py`
  - 一键（可选）：`MMDET3D/local/ros/scripts/run_bag_node.sh`
- FAST-LIO：`Scripts/run_fastlio_mapping.sh`
- Pose Bridge：`Scripts/run_fastlio_pose_bridge.sh`
- MCTrack Online：`Scripts/run_mctrack_online_node.sh`
- RViz：`Scripts/run_rviz.sh`

## 备注（避免接口误解）

- `mctrack_online_node.py` 默认订阅 **`/kitti/detections`（Detection3DArray）**。
- `PointPillars` 同时也会发布 **`/detection/kitti_tracking`（String）**，但该在线节点默认不消费该话题。


