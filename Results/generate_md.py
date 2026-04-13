import os
import json

base_dir = r"d:\OneDrive\NUS\ME5400 学期项目\ME5400\Results"
out_file = os.path.join(base_dir, "EXPERIMENT_TABLE_AC_CN.md")

# Find all sequence directories
seqs = []
for entry in os.listdir(base_dir):
    if entry.endswith("_results") and os.path.isdir(os.path.join(base_dir, entry)):
        seqs.append(entry.split("_")[0])
seqs.sort()

# Also check which ones actually have Online/metrics.json
valid_seqs = []
for seq in seqs:
    json_path = os.path.join(base_dir, f"{seq}_results", "Online", "metrics.json")
    if os.path.exists(json_path) and seq not in ["0012", "0017"]:
        valid_seqs.append(seq)

def get_color_span(val_str, val_float):
    if val_float < 0:
        return f'<span style="color: #c62828;">{val_str}</span>'
    elif val_float > 0:
        return f'<span style="color: #2e7d32;">{val_str}</span>'
    else:
        return val_str

md_content = """# ME5400 实验汇总表（三轨在线对比）

## 对比设置
- `Baseline`: 单独 FAST-LIO
- `FAST-LIO+MCTrack`: 在线动态权重 + 跟踪前端
- `Joint Backend`: 在 `FAST-LIO+MCTrack` 基础上接入联合后端
- 指标来源：`Results/<SEQ>_results/Online/metrics.json`
- 已完成在线结果序列：`""" + ", ".join(valid_seqs) + """`

## ATE / RPE 汇总
| 序列 | Baseline ATE RMSE | FAST-LIO+MCTrack ATE RMSE | 相对 Baseline | Joint Backend ATE RMSE | 相对 Baseline | Baseline RPE RMSE | FAST-LIO+MCTrack RPE RMSE | Joint Backend RPE RMSE | 最优 ATE |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---|
"""

table2 = """
## ATE 细表
| 序列 | Baseline 平均/最大/帧数 | FAST-LIO+MCTrack 平均/最大/帧数 | Joint Backend 平均/最大/帧数 |
|---|---|---|---|
"""

table3 = """
## KITTI 相对误差汇总
| 序列 | 实际段长 (m) | 最短有效匹配里程 (m) | Baseline Tr (%) | FAST-LIO+MCTrack Tr (%) | 相对 Baseline | Joint Backend Tr (%) | 相对 Baseline | Baseline Rot | FAST-LIO+MCTrack Rot | 相对 Baseline | Joint Backend Rot | 相对 Baseline | 分段数(B/F/J) |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|
"""

for seq in valid_seqs:
    json_path = os.path.join(base_dir, f"{seq}_results", "Online", "metrics.json")
    with open(json_path, "r", encoding="utf-8") as f:
        data = json.load(f)
    
    traj_data = {t["name"]: t for t in data.get("ate_rpe", {}).get("trajectories", [])}
    if not traj_data:
        continue
        
    b_ate = traj_data.get("Baseline", {}).get("ate", {}).get("ATE_RMSE", None)
    f_ate = traj_data.get("FAST-LIO+MCTrack", {}).get("ate", {}).get("ATE_RMSE", None)
    j_ate = traj_data.get("Joint Backend", {}).get("ate", {}).get("ATE_RMSE", None)
    
    if b_ate is None: continue
    
    improvements = {i["name"]: i["ate_improvement_percent"] for i in data.get("ate_rpe", {}).get("improvements_relative_to_first", [])}
    f_rel = improvements.get("FAST-LIO+MCTrack", 0.0)
    j_rel = improvements.get("Joint Backend", 0.0)
    
    f_rel_str = f"{f_rel:+.2f}%"
    j_rel_str = f"{j_rel:+.2f}%"
    
    b_rpe = traj_data.get("Baseline", {}).get("rpe", {}).get("RPE_RMSE", 0.0)
    f_rpe = traj_data.get("FAST-LIO+MCTrack", {}).get("rpe", {}).get("RPE_RMSE", 0.0)
    j_rpe = traj_data.get("Joint Backend", {}).get("rpe", {}).get("RPE_RMSE", 0.0)
    
    ates = [("Baseline", b_ate), ("FAST-LIO+MCTrack", f_ate), ("Joint Backend", j_ate)]
    best_ate = min((a for a in ates if a[1] is not None), key=lambda x: x[1])[0]
    
    md_content += f"| {seq} | {b_ate:.4f} | {f_ate:.4f} | {get_color_span(f_rel_str, f_rel)} | {j_ate:.4f} | {get_color_span(j_rel_str, j_rel)} | {b_rpe:.4f} | {f_rpe:.4f} | {j_rpe:.4f} | {best_ate} |\n"
    
    # Table 2
    def get_t2_str(name):
        d = traj_data.get(name, {})
        if not d: return "N/A"
        return f"{d.get('ate', {}).get('ATE_mean', 0):.4f} / {d.get('ate', {}).get('ATE_max', 0):.4f} / {d.get('matched_frames', 0)}"
        
    table2 += f"| {seq} | {get_t2_str('Baseline')} | {get_t2_str('FAST-LIO+MCTrack')} | {get_t2_str('Joint Backend')} |\n"
    
    # Table 3
    kd = data.get("kitti_tr_rot", {})
    rows = {r["name"]: r for r in kd.get("rows", [])}
    lens = kd.get("lengths_m", [])
    len_str = ", ".join(map(str, lens)) if lens else "N/A"
    
    short_span = kd.get("min_matched_span_m", 0.0)
    
    if "Baseline" not in rows or not lens:
        table3 += f"| {seq} | N/A | {short_span:.3f} | N/A | N/A | N/A | N/A | N/A | N/A | N/A | N/A | N/A | N/A | 0/0/0 |\n"
    else:
        b_r = rows["Baseline"]
        f_r = rows.get("FAST-LIO+MCTrack", {})
        j_r = rows.get("Joint Backend", {})
        
        b_tr = b_r.get("Tr_percent", 0.0)
        f_tr = f_r.get("Tr_percent", 0.0)
        j_tr = j_r.get("Tr_percent", 0.0)
        f_tr_rel = f_r.get("Tr_gain_percent", 0.0)
        j_tr_rel = j_r.get("Tr_gain_percent", 0.0)
        
        b_rot = b_r.get("Rot_deg_per_100m", 0.0)
        f_rot = f_r.get("Rot_deg_per_100m", 0.0)
        j_rot = j_r.get("Rot_deg_per_100m", 0.0)
        f_rot_rel = f_r.get("Rot_gain_percent", 0.0)
        j_rot_rel = j_r.get("Rot_gain_percent", 0.0)
        
        b_seg = int(b_r.get("n_segments", 0))
        f_seg = int(f_r.get("n_segments", 0))
        j_seg = int(j_r.get("n_segments", 0))
        
        f_tr_rel_str = f"{f_tr_rel:+.2f}%"
        j_tr_rel_str = f"{j_tr_rel:+.2f}%"
        f_rot_rel_str = f"{f_rot_rel:+.2f}%"
        j_rot_rel_str = f"{j_rot_rel:+.2f}%"
        
        table3 += f"| {seq} | {len_str} | {short_span:.3f} | {b_tr:.4f} | {f_tr:.4f} | {get_color_span(f_tr_rel_str, f_tr_rel)} | {j_tr:.4f} | {get_color_span(j_tr_rel_str, j_tr_rel)} | {b_rot:.4f} | {f_rot:.4f} | {get_color_span(f_rot_rel_str, f_rot_rel)} | {j_rot:.4f} | {get_color_span(j_rot_rel_str, j_rot_rel)} | {b_seg}/{f_seg}/{j_seg} |\n"

md_content += table2 + table3 + """
## 备注
- `0016` 的有效匹配里程过短，`Tr / Rot` 无可用段长，因此表中显示为 `N/A`。
- 如果某序列没有对应的 `Online/metrics.json`，则未纳入三轨在线汇总。
"""

with open(out_file, "w", encoding="utf-8") as f:
    f.write(md_content)
print("Updated markdown successfully.")
