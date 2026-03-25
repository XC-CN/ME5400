# ME5400 项目 vs SLAMMOT 最新论文 全面对比

## 对比对象概览

| # | 论文简称 | 全称 | 发表时间 | 发表渠道 |
|---|---------|------|---------|---------|
| ① | **Conf SLAMMOT** | LiDAR SLAMMOT based on Confidence-guided Data Association | 2024.12 | arXiv |
| ② | **LIMOT** | A Tightly-Coupled System for LiDAR-Inertial Odometry and Multi-Object Tracking | 2024.07 | IEEE RA-L |
| ③ | **IMM-SLAMMOT** | Visual SLAMMOT Considering Multiple Motion Models | 2024.11 | arXiv (2411.19134) |
| ④ | **Online Dyn SLAM** | Online Dynamic SLAM with Incremental Smoothing and Mapping | 2025.09 | arXiv |
| ⑤ | **DL-SLOT** | Tightly-Coupled Dynamic LiDAR SLAM and 3D Object Tracking Based on Collaborative Graph Optimization | 2024 | IEEE T-IV |
| 🔵 | **ME5400（本项目）** | FAST-LIO + PointPillars + MCTrack 双向协同管线 | 进行中 | 学期项目 |

---

## 一、系统架构对比

| 维度 | Conf SLAMMOT | LIMOT | IMM-SLAMMOT | Online Dyn SLAM | DL-SLOT | **ME5400** |
|------|-------------|-------|-------------|-----------------|---------|-----------|
| **传感器** | LiDAR | LiDAR + IMU | 立体相机 | 立体相机 | LiDAR | **LiDAR + IMU** |
| **里程计前端** | LiDAR 里程计 | 自研 LIO | ORB-SLAM2 | 自研 VO | LiDAR 里程计 | **FAST-LIO (IESKF)** |
| **3D 检测器** | 外部输入 | 外部输入 | 2D 检测 | 实例分割 | 外部输入 | **PointPillars (MMDet3D)** |
| **跟踪器** | 联合因子图 | 滑窗 MOT | IMM 滤波 | 因子图内跟踪 | 滑窗关联 | **MCTrack** |
| **后端优化** | 因子图 | 因子图 | 因子图 (BA) | iSAM2 因子图 | 图优化 | **无（各模块独立）** |
| **ROS 集成** | 未提及 | 未提及 | 无 | 无 | 未提及 | **✅ 完整 ROS 管线** |

---

## 二、耦合方式对比

| 维度 | Conf SLAMMOT | LIMOT | IMM-SLAMMOT | Online Dyn SLAM | DL-SLOT | **ME5400** |
|------|-------------|-------|-------------|-----------------|---------|-----------|
| **SLAM ↔ MOT 耦合** | 紧耦合 | 紧耦合 | 紧耦合 | 紧耦合 | 紧耦合 | **松耦合** |
| **耦合机制** | 联合因子图 | 滑窗因子图 | BA 联合优化 | iSAM2 增量优化 | 协同图优化 | **topic 通信 + 降权** |
| **自车 → 目标** | ✅ 联合优化 | ✅ 联合优化 | ✅ 联合优化 | ✅ 联合优化 | ✅ 联合优化 | ✅ 位姿传递 |
| **目标 → 自车** | ✅ 联合优化 | ✅ 动态特征过滤 | ✅ 联合优化 | ✅ 联合优化 | ✅ 动态点过滤 | ⚠️ 仅降权（单向） |
| **回头修正能力** | ✅ 窗口内 | ✅ 窗口内 | ✅ BA 内 | ✅ 增量式 | ✅ 窗口内 | ❌ 无 |

---

## 三、动态物体处理策略对比

| 策略 | Conf SLAMMOT | LIMOT | IMM-SLAMMOT | Online Dyn SLAM | DL-SLOT | **ME5400** |
|------|-------------|-------|-------------|-----------------|---------|-----------|
| **动态点处理** | 过滤 + 跟踪 | 过滤（不参与建图） | 分离静/动态点 | 联合估计 | 过滤 + 跟踪 | **降权（仍参与建图）** |
| **"化毒瘤为路标"** | ❌ | ❌ | ❌ | ✅ 部分 | ❌ | **❌（但计划升级 ✅）** |
| **刚体约束利用** | ❌ | ❌ | ❌ | ❌ | ❌ | **❌（但计划升级 ✅）** |
| **动态目标速度利用** | ✅ 隐式 | ✅ 因子 | ✅ IMM 多模型 | ✅ 因子 | ✅ 运动模型 | **✅ MCTrack 提供** |

> [!IMPORTANT]
> **关键发现**：5 篇论文中，没有一篇真正实现了"化动态物体为路标"的刚体约束利用。这意味着你的升级方向如果做到这一点，将是**真正的创新 contribution**。

---

## 四、因子图设计对比

| 因子类型 | Conf SLAMMOT | LIMOT | IMM-SLAMMOT | Online Dyn SLAM | DL-SLOT | **ME5400（升级后）** |
|---------|-------------|-------|-------------|-----------------|---------|-------------------|
| **里程计因子** | ✅ LiDAR | ✅ LiDAR | ✅ VO | ✅ VO | ✅ LiDAR | **✅ FAST-LIO 输出** |
| **IMU 预积分因子** | ❌ | ✅ | ❌ | ❌ | ❌ | **✅（可选）** |
| **目标检测因子** | ✅ | ✅ | ✅ | ✅ | ✅ | **✅ PointPillars** |
| **运动模型因子** | ✅ 隐式 | ✅ | ✅ IMM 多模型 | ✅ | ✅ | **✅ MCTrack 信息** |
| **刚体约束因子** | ❌ | ❌ | ❌ | ❌ | ❌ | **✅（创新点！）** |
| **数据关联** | 置信度隐式关联 | 滑窗历史轨迹 | IMM 匹配 | 实例 ID | 滑窗匹配 | **MCTrack tracking ID** |
| **优化器** | 未指定 | GTSAM / g2o | g2o (BA) | GTSAM iSAM2 | g2o | **GTSAM（建议）** |

---

## 五、数据关联策略对比

| 维度 | Conf SLAMMOT | LIMOT | IMM-SLAMMOT | Online Dyn SLAM | DL-SLOT | **ME5400** |
|------|-------------|-------|-------------|-----------------|---------|-----------|
| **关联方式** | 置信度隐式关联 | 滑窗匹配 | 匈牙利算法 | 实例分割 ID | 滑窗 IoU | **MCTrack tracking ID** |
| **创新点** | 关联嵌入优化过程 | 基于轨迹历史 | 多模型自适应 | 语义级关联 | 协同优化 | **已有稳定 ID，直接可用** |
| **漏检处理** | ✅ 置信度权重 | ⚠️ 一般 | ✅ IMM 预测 | ⚠️ 一般 | ⚠️ 一般 | **✅ MCTrack 内置** |

---

## 六、性能与实时性对比

| 维度 | Conf SLAMMOT | LIMOT | IMM-SLAMMOT | Online Dyn SLAM | DL-SLOT | **ME5400** |
|------|-------------|-------|-------------|-----------------|---------|-----------|
| **实时性** | ✅ | ✅ | ✅ | ✅（5x 加速） | ✅ | **✅** |
| **优化方式** | 批量滑窗 | 批量滑窗 | BA | iSAM2 增量 | 批量滑窗 | **无后端优化** |
| **可扩展性** | 中 | 中 | 中 | **高（增量式）** | 中 | **高（模块化 ROS）** |
| **评估数据集** | KITTI | KITTI | KITTI | 自有数据集 | KITTI | **KITTI** |

---

## 七、各论文核心创新点 vs ME5400 对比

| 论文 | 核心创新 | ME5400 现状 | ME5400 能否借鉴 |
|------|---------|------------|----------------|
| **Conf SLAMMOT** | 置信度引导的隐式数据关联 → 关联嵌入因子图优化 | MCTrack 已提供置信度 | ✅ 可利用 MCTrack 的置信度作为因子权重 |
| **LIMOT** | LiDAR-Inertial + MOT 紧耦合因子图 | FAST-LIO + MCTrack 松耦合 | ✅ 直接参考其因子图结构 |
| **IMM-SLAMMOT** | 交互式多运动模型自适应切换 | MCTrack 单一运动模型 | ✅ 可在因子图中加入多运动模型因子 |
| **Online Dyn SLAM** | iSAM2 增量式动态 SLAM，5x 加速 | 无后端优化 | ✅ 用 iSAM2 替代批量优化 |
| **DL-SLOT** | 滑动窗口协同图优化 | 无图优化 | ✅ 参考其滑动窗口设计 |

---

## 八、ME5400 的独特优势

| 优势 | 详细说明 | 哪篇论文没有 |
|------|---------|-------------|
| **FAST-LIO IESKF 前端** | LiDAR-Inertial 里程计天花板，高频 IMU 融合（200Hz+），精度极高 | 全部（LIMOT 自研前端，其余用更简单的里程计） |
| **PointPillars 端到端检测** | 完整的 3D 目标检测管线，已集成 ROS | 全部（均假设外部检测器输入） |
| **MCTrack 丰富跟踪信息** | 速度、加速度、置信度、tracking ID 一应俱全 | 全部（跟踪信息不如 MCTrack 丰富） |
| **完整 ROS 生态** | 一键启动脚本、RViz 可视化、topic 通信 | 全部（均为离线或自研框架） |
| **"化毒瘤为路标"方向** | 将动态车辆作为移动路标提供额外定位约束 | **全部（没有一篇真正做到）** |

---

## 九、升级路线建议

基于以上对比，ME5400 升级的最优方案是**博采众长**：

```
从 LIMOT 借鉴：     因子图结构（自车位姿 + IMU偏置 + 目标位姿 联合优化）
从 Conf SLAMMOT 借鉴：置信度引导的因子加权（利用 MCTrack 的置信度）
从 Online Dyn SLAM 借鉴：iSAM2 增量优化（保证实时性和可扩展性）
从 IMM-SLAMMOT 借鉴：  多运动模型因子（提升对复杂运动目标的跟踪）
从 DL-SLOT 借鉴：     滑动窗口管理与边缘化策略

ME5400 独有 contribution：刚体约束因子 —— 利用动态车辆表面点云的刚体特性作为定位约束
```

### 升级后的论文定位

**题目建议**：*Dynamic is not a Curse: Joint Ego-Motion and Multi-Object Tracking via Object-Centric LIO*

**Contribution 总结**：
1. 在 FAST-LIO 之上构建滑动窗口因子图，首次实现 LiDAR-Inertial 里程计与多目标跟踪的 Object-Centric 联合优化
2. 提出刚体约束因子，将动态车辆从"干扰源"转变为"移动路标"，在几何退化场景中提供额外定位约束
3. 利用 MCTrack 的置信度信息作为因子权重，实现自适应的数据关联

**与现有工作的差异化**：
- vs LIMOT：增加了刚体约束因子，动态物体不仅被过滤更被利用
- vs Conf SLAMMOT：基于 FAST-LIO 的高质量 LiDAR-Inertial 前端，精度更高
- vs IMM-SLAMMOT：从视觉扩展到 LiDAR-Inertial，传感器精度更高
- vs Online Dyn SLAM：从相机扩展到 LiDAR + IMU，并引入刚体约束
- vs DL-SLOT：增加 IMU 融合 + 刚体约束 + 置信度加权



# "化动态物体为移动路标" 相关论文

### 已有工作（确实有人做了）

#### 1. VDO-SLAM: A Visual Dynamic Object-aware SLAM System（2020）
- **arXiv**：[2005.11052](https://arxiv.org/abs/2005.11052)
- **已开源**：[GitHub](https://github.com/halajun/VDO_SLAM)
- **做了什么**：将动态物体的 SE(3) 运动纳入统一因子图，联合估计相机位姿 + 静态地图 + 动态物体运动，动态物体上的特征点**参与优化而非被丢弃**
- **传感器**：RGB-D / 立体相机
- **局限**：❌ 视觉方案，不是 LiDAR；没有 IMU 融合；没有显式的刚体点云约束

#### 2. Exploiting Rigid Body Motion for SLAM in Dynamic Environments
- **来源**：University of Michigan
- **做了什么**：提出在因子图中利用刚体假设——动态物体上的点在物体坐标系下是固定的，将这个约束作为因子加入图优化
- **核心数学**：引入"body-fixed frame"变量变换，不需要显式估计刚体的位姿和3D模型
- **局限**：❌ 2D 仿真验证为主；没有 LiDAR 实验

#### 3. Dynamic SLAM: The Need For Speed（2020）
- **arXiv**：[2002.08584](https://arxiv.org/abs/2002.08584)
- **做了什么**：利用语义分割估计刚体运动，生成动态+静态的完整地图，可以提取动态物体速度
- **局限**：❌ 视觉方案；没有显式把动态物体作为定位约束

#### 4. DynaSLAM II: Tightly-Coupled Multi-Object Tracking and SLAM（2020）
- **arXiv**：[2010.07820](https://arxiv.org/abs/2010.07820)
- **做了什么**：在 Bundle Adjustment 中联合优化静态结构 + 动态物体 + 相机轨迹
- **局限**：❌ 视觉方案（RGB-D/立体）；动态物体的特征点仍然主要用于跟踪而非增强定位

---

### 关键发现：谁做了，谁没做

| 能力 | VDO-SLAM | Rigid Body Motion | DynaSLAM II | Dynamic SLAM | **你的升级方向** |
|------|----------|-------------------|-------------|-------------|-----------------|
| 动态物体参与优化 | ✅ | ✅ | ✅ | ✅ | ✅ |
| 刚体运动约束 | ⚠️ 隐式 | ✅ 显式 | ❌ | ⚠️ 隐式 | **✅ 显式** |
| **LiDAR 点云**刚体约束 | ❌ 视觉 | ❌ 2D仿真 | ❌ 视觉 | ❌ 视觉 | **✅** |
| LiDAR-Inertial 融合 | ❌ | ❌ | ❌ | ❌ | **✅ FAST-LIO** |
| 动态物体增强自车定位 | ⚠️ 理论上可以 | ✅ 理论证明 | ❌ | ❌ | **✅ 目标** |
| 实车/实际数据集验证 | ✅ KITTI | ❌ 仿真 | ✅ TUM | ✅ 自有 | **✅ KITTI** |
| 结合 MOT 跟踪器 | ❌ 自研 | ❌ | ✅ 自研 | ❌ | **✅ MCTrack** |

---

## 💡 结论

**"化动态物体为移动路标"这个思想确实有人提过，但存在明显的空白地带：**

```
已有工作覆盖：
  ✅ 视觉 SLAM + 动态物体参与优化（VDO-SLAM, DynaSLAM II）
  ✅ 2D 仿真中的刚体约束因子（Rigid Body Motion）
  ✅ 视觉场景中的速度估计（Dynamic SLAM）

尚未被覆盖的空白（= 你的创新空间）：
  ❌ LiDAR 点云 + 刚体约束因子（没人做过）
  ❌ LiDAR-Inertial + 动态物体路标（没人做过）
  ❌ 成熟 MOT 跟踪器 + 因子图联合优化（没人做过）
  ❌ 在 FAST-LIO 之上叠加 object-centric 因子图（没人做过）
```

所以更准确的说法是：

> **思想不是全新的**（VDO-SLAM 2020 年就提出了），但**在 LiDAR-Inertial 领域没有人真正实现过**。视觉方案和 LiDAR 方案的差异很大（点云刚体约束的定义完全不同），这就是你的差异化创新点。

你可以在论文 related work 中引用 VDO-SLAM 和 Rigid Body Motion，说明思想来源，然后强调你是**首次在 LiDAR-Inertial 框架下实现 object-centric 联合优化 + 刚体点云约束**。这样既诚实又有创新性。