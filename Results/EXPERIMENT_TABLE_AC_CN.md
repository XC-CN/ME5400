# ME5400 实验汇总表（三轨在线对比）

## 对比设置
- `Baseline`: 单独 FAST-LIO
- `FAST-LIO+MCTrack`: 在线动态权重 + 跟踪前端
- `Joint Backend`: 在 `FAST-LIO+MCTrack` 基础上接入联合后端
- 指标来源：`Results/<SEQ>_results/Online/metrics.json`
- 已完成在线结果序列：`0003, 0006, 0007, 0009, 0010, 0013, 0020`

## ATE / RPE 汇总
| 序列 | Baseline ATE RMSE | FAST-LIO+MCTrack ATE RMSE | 相对 Baseline | Joint Backend ATE RMSE | 相对 Baseline | Baseline RPE RMSE | FAST-LIO+MCTrack RPE RMSE | Joint Backend RPE RMSE | 最优 ATE |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---|
| 0003 | 0.5294 | 0.4513 | <span style="color: #2e7d32;">+14.75%</span> | 0.5875 | <span style="color: #c62828;">-10.98%</span> | 0.2049 | 0.1809 | 0.2393 | FAST-LIO+MCTrack |
| 0006 | 0.5238 | 0.4710 | <span style="color: #2e7d32;">+10.09%</span> | 0.5576 | <span style="color: #c62828;">-6.45%</span> | 0.1324 | 0.0869 | 0.1677 | FAST-LIO+MCTrack |
| 0007 | 2.3128 | 2.2393 | <span style="color: #2e7d32;">+3.18%</span> | 2.2393 | <span style="color: #2e7d32;">+3.18%</span> | 0.1716 | 0.1683 | 0.1683 | FAST-LIO+MCTrack |
| 0009 | 1.6831 | 1.6675 | <span style="color: #2e7d32;">+0.93%</span> | 3.3082 | <span style="color: #c62828;">-96.56%</span> | 0.1787 | 0.1651 | 1.9514 | FAST-LIO+MCTrack |
| 0010 | 3.7651 | 0.8680 | <span style="color: #2e7d32;">+76.95%</span> | 0.9632 | <span style="color: #2e7d32;">+74.42%</span> | 0.2774 | 0.2281 | 0.2334 | FAST-LIO+MCTrack |
| 0013 | 0.6047 | 0.5816 | <span style="color: #2e7d32;">+3.82%</span> | 0.5816 | <span style="color: #2e7d32;">+3.82%</span> | 0.1495 | 0.1392 | 0.1392 | Joint Backend |
| 0020 | 11.7323 | 6.7938 | <span style="color: #2e7d32;">+42.09%</span> | 6.8829 | <span style="color: #2e7d32;">+41.33%</span> | 0.2250 | 0.2245 | 0.2270 | FAST-LIO+MCTrack |

## ATE 细表
| 序列 | Baseline 平均/最大/帧数 | FAST-LIO+MCTrack 平均/最大/帧数 | Joint Backend 平均/最大/帧数 |
|---|---|---|---|
| 0003 | 0.4744 / 1.0819 / 129 | 0.3886 / 0.9261 / 129 | 0.4599 / 2.3362 / 129 |
| 0006 | 0.3273 / 1.7698 / 254 | 0.2757 / 1.5796 / 254 | 0.3343 / 2.4547 / 254 |
| 0007 | 2.1821 / 3.9662 / 784 | 2.1022 / 3.8504 / 784 | 2.1022 / 3.8504 / 784 |
| 0009 | 1.5520 / 2.7081 / 787 | 1.5317 / 2.8659 / 788 | 1.7280 / 40.3063 / 788 |
| 0010 | 3.7246 / 5.5684 / 279 | 0.8063 / 1.9093 / 279 | 0.9078 / 1.9335 / 279 |
| 0013 | 0.5683 / 1.4489 / 324 | 0.5391 / 1.5361 / 324 | 0.5391 / 1.5361 / 324 |
| 0020 | 10.7291 / 17.0888 / 821 | 6.0536 / 11.6802 / 821 | 6.1244 / 11.8018 / 821 |

## KITTI 相对误差汇总
| 序列 | 实际段长 (m) | 最短有效匹配里程 (m) | Baseline Tr (%) | FAST-LIO+MCTrack Tr (%) | 相对 Baseline | Joint Backend Tr (%) | 相对 Baseline | Baseline Rot | FAST-LIO+MCTrack Rot | 相对 Baseline | Joint Backend Rot | 相对 Baseline | 分段数(B/F/J) |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|
| 0003 | 20, 40, 60, 80, 100, 150 | 153.375 | 1.4794 | 1.3516 | <span style="color: #2e7d32;">+8.63%</span> | 1.6856 | <span style="color: #c62828;">-13.94%</span> | 1.1509 | 1.1569 | <span style="color: #c62828;">-0.52%</span> | 1.3699 | <span style="color: #c62828;">-19.03%</span> | 44/44/44 |
| 0006 | 20, 40 | 49.250 | 4.1614 | 4.0605 | <span style="color: #2e7d32;">+2.42%</span> | 4.3990 | <span style="color: #c62828;">-5.71%</span> | 12.8416 | 11.6291 | <span style="color: #2e7d32;">+9.44%</span> | 16.0675 | <span style="color: #c62828;">-25.12%</span> | 44/44/44 |
| 0007 | 20, 40, 60, 80, 100, 150, 200 | 511.560 | 3.9042 | 3.9267 | <span style="color: #c62828;">-0.57%</span> | 3.9267 | <span style="color: #c62828;">-0.57%</span> | 4.9702 | 5.0234 | <span style="color: #c62828;">-1.07%</span> | 5.0234 | <span style="color: #c62828;">-1.07%</span> | 425/425/425 |
| 0009 | 20, 40, 60, 80, 100, 150, 200 | 706.766 | 2.5630 | 2.5281 | <span style="color: #2e7d32;">+1.36%</span> | 4.2200 | <span style="color: #c62828;">-64.65%</span> | 3.2425 | 3.2294 | <span style="color: #2e7d32;">+0.40%</span> | 3.7550 | <span style="color: #c62828;">-15.81%</span> | 452/452/452 |
| 0010 | 20, 40, 60, 80, 100, 150, 200 | 399.675 | 1.3884 | 0.9773 | <span style="color: #2e7d32;">+29.61%</span> | 1.1140 | <span style="color: #2e7d32;">+19.76%</span> | 1.1274 | 1.1368 | <span style="color: #c62828;">-0.83%</span> | 1.5713 | <span style="color: #c62828;">-39.38%</span> | 153/153/153 |
| 0013 | 20, 40, 60, 80, 100, 150 | 186.820 | 2.4374 | 2.3741 | <span style="color: #2e7d32;">+2.60%</span> | 2.3741 | <span style="color: #2e7d32;">+2.60%</span> | 2.3026 | 2.2419 | <span style="color: #2e7d32;">+2.64%</span> | 2.2419 | <span style="color: #2e7d32;">+2.64%</span> | 121/121/121 |
| 0020 | 20, 40, 60, 80, 100, 150, 200 | 707.825 | 3.2245 | 2.3967 | <span style="color: #2e7d32;">+25.67%</span> | 2.4302 | <span style="color: #2e7d32;">+24.63%</span> | 1.1894 | 1.1748 | <span style="color: #2e7d32;">+1.22%</span> | 1.2326 | <span style="color: #c62828;">-3.63%</span> | 479/479/479 |

## 备注
- `0016` 的有效匹配里程过短，`Tr / Rot` 无可用段长，因此表中显示为 `N/A`。
- 如果某序列没有对应的 `Online/metrics.json`，则未纳入三轨在线汇总。
