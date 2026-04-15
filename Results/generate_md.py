#!/usr/bin/env python3
from __future__ import annotations

import json
from pathlib import Path


BASE_DIR = Path(__file__).resolve().parent
OUT_FILE = BASE_DIR / "EXPERIMENT_TABLE_AC_CN.md"
SKIP_SEQS = {"0012", "0017"}


def color_span(text: str, value: float | None) -> str:
    if value is None:
        return text
    if value < 0:
        return f'<span style="color: #c62828;">{text}</span>'
    if value > 0:
        return f'<span style="color: #2e7d32;">{text}</span>'
    return text


def fmt(value: float | None, digits: int = 4) -> str:
    if value is None:
        return "N/A"
    return f"{value:.{digits}f}"


def fmt_signed_percent(value: float | None, digits: int = 2) -> str:
    if value is None:
        return "N/A"
    return f"{value:+.{digits}f}%"


def load_metrics(seq: str) -> dict | None:
    path = BASE_DIR / f"{seq}_results" / "Online" / "metrics.json"
    if not path.exists():
        return None
    return json.loads(path.read_text(encoding="utf-8"))


def get_seq_ids() -> list[str]:
    seqs = []
    for entry in BASE_DIR.iterdir():
        if entry.is_dir() and entry.name.endswith("_results"):
            seq = entry.name.split("_", 1)[0]
            if seq not in SKIP_SEQS and (entry / "Online" / "metrics.json").exists():
                seqs.append(seq)
    return sorted(seqs)


def traj_map(data: dict) -> dict[str, dict]:
    return {item["name"]: item for item in data.get("ate_rpe", {}).get("trajectories", [])}


def improvement_map(data: dict) -> dict[str, float]:
    return {
        item["name"]: item.get("ate_improvement_percent")
        for item in data.get("ate_rpe", {}).get("improvements_relative_to_first", [])
    }


def kitti_row_map(data: dict) -> dict[str, dict]:
    return {item["name"]: item for item in data.get("kitti_tr_rot", {}).get("rows", [])}


def build_markdown() -> str:
    valid_seqs = get_seq_ids()

    lines = [
        "# ME5400 实验汇总表（三轨在线对比）",
        "",
        "## 对比设置",
        "- `Baseline`: 单独 FAST-LIO",
        "- `FAST-LIO+MCTrack`: 在线动态权重 + 跟踪前端",
        "- `Joint Backend`: 在 `FAST-LIO+MCTrack` 基础上接入联合后端",
        "- 指标来源：`Results/<SEQ>_results/Online/metrics.json`",
        f"- 已完成在线结果序列：`{', '.join(valid_seqs)}`",
        "",
        "## ATE / RPE 汇总",
        "| 序列 | Baseline ATE RMSE | FAST-LIO+MCTrack ATE RMSE | 相对 Baseline | Joint Backend ATE RMSE | 相对 Baseline | Baseline RPE RMSE | FAST-LIO+MCTrack RPE RMSE | Joint Backend RPE RMSE | 最优 ATE |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---|",
    ]

    table2 = [
        "",
        "## ATE 细表",
        "| 序列 | Baseline 平均/最大/帧数 | FAST-LIO+MCTrack 平均/最大/帧数 | Joint Backend 平均/最大/帧数 |",
        "|---|---|---|---|",
    ]

    table3 = [
        "",
        "## KITTI 相对误差汇总",
        "| 序列 | 实际段长 (m) | 最短有效匹配里程 (m) | Baseline Tr (%) | FAST-LIO+MCTrack Tr (%) | 相对 Baseline | Joint Backend Tr (%) | 相对 Baseline | Baseline Rot | FAST-LIO+MCTrack Rot | 相对 Baseline | Joint Backend Rot | 相对 Baseline | 分段数(B/F/J) |",
        "|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|",
    ]

    for seq in valid_seqs:
        data = load_metrics(seq)
        if data is None:
            continue

        trajs = traj_map(data)
        improvements = improvement_map(data)

        baseline = trajs.get("Baseline")
        front = trajs.get("FAST-LIO+MCTrack")
        joint = trajs.get("Joint Backend")
        if not baseline or not front or not joint:
            continue

        b_ate = baseline.get("ate", {}).get("ATE_RMSE")
        f_ate = front.get("ate", {}).get("ATE_RMSE")
        j_ate = joint.get("ate", {}).get("ATE_RMSE")
        b_rpe = baseline.get("rpe", {}).get("RPE_RMSE")
        f_rpe = front.get("rpe", {}).get("RPE_RMSE")
        j_rpe = joint.get("rpe", {}).get("RPE_RMSE")
        f_rel = improvements.get("FAST-LIO+MCTrack")
        j_rel = improvements.get("Joint Backend")

        best_ate = min(
            [
                ("Baseline", b_ate),
                ("FAST-LIO+MCTrack", f_ate),
                ("Joint Backend", j_ate),
            ],
            key=lambda item: item[1],
        )[0]

        lines.append(
            f"| {seq} | {fmt(b_ate)} | {fmt(f_ate)} | {color_span(fmt_signed_percent(f_rel), f_rel)} | "
            f"{fmt(j_ate)} | {color_span(fmt_signed_percent(j_rel), j_rel)} | {fmt(b_rpe)} | {fmt(f_rpe)} | {fmt(j_rpe)} | {best_ate} |"
        )

        def ate_detail(item: dict) -> str:
            ate = item.get("ate", {})
            return f"{fmt(ate.get('ATE_mean'))} / {fmt(ate.get('ATE_max'))} / {item.get('matched_frames', 0)}"

        table2.append(
            f"| {seq} | {ate_detail(baseline)} | {ate_detail(front)} | {ate_detail(joint)} |"
        )

        kitti = data.get("kitti_tr_rot", {})
        row_map = kitti_row_map(data)
        lengths = kitti.get("lengths_m", [])
        len_str = ", ".join(str(int(v)) if float(v).is_integer() else str(v) for v in lengths) if lengths else "N/A"
        min_span = kitti.get("min_matched_span_m")

        b_row = row_map.get("Baseline")
        f_row = row_map.get("FAST-LIO+MCTrack")
        j_row = row_map.get("Joint Backend")
        if not b_row or not f_row or not j_row or not lengths:
            table3.append(
                f"| {seq} | {len_str} | {fmt(min_span, 3)} | N/A | N/A | N/A | N/A | N/A | N/A | N/A | N/A | N/A | N/A | 0/0/0 |"
            )
            continue

        table3.append(
            f"| {seq} | {len_str} | {fmt(min_span, 3)} | "
            f"{fmt(b_row.get('Tr_percent'))} | {fmt(f_row.get('Tr_percent'))} | {color_span(fmt_signed_percent(f_row.get('Tr_gain_percent')), f_row.get('Tr_gain_percent'))} | "
            f"{fmt(j_row.get('Tr_percent'))} | {color_span(fmt_signed_percent(j_row.get('Tr_gain_percent')), j_row.get('Tr_gain_percent'))} | "
            f"{fmt(b_row.get('Rot_deg_per_100m'))} | {fmt(f_row.get('Rot_deg_per_100m'))} | {color_span(fmt_signed_percent(f_row.get('Rot_gain_percent')), f_row.get('Rot_gain_percent'))} | "
            f"{fmt(j_row.get('Rot_deg_per_100m'))} | {color_span(fmt_signed_percent(j_row.get('Rot_gain_percent')), j_row.get('Rot_gain_percent'))} | "
            f"{int(b_row.get('n_segments', 0))}/{int(f_row.get('n_segments', 0))}/{int(j_row.get('n_segments', 0))} |"
        )

    lines.extend(
        table2
        + table3
        + [
            "",
            "## 备注",
            "- `0016` 的有效匹配里程过短，`Tr / Rot` 无可用段长，因此表中显示为 `N/A`。",
            "- 如果某序列没有对应的 `Online/metrics.json`，则未纳入三轨在线汇总。",
        ]
    )
    return "\n".join(lines) + "\n"


def main() -> None:
    OUT_FILE.write_text(build_markdown(), encoding="utf-8")
    print(f"Updated markdown successfully: {OUT_FILE}")


if __name__ == "__main__":
    main()
