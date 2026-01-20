#!/usr/bin/env python3
"""将 KITTI tracking 标注转为 MCTrack 可读取的检测格式。"""
from pathlib import Path
import os

ROOT = Path(__file__).resolve().parents[2] / "MCTrack" / "data" / "kitti"
LABEL_DIR = ROOT / "datasets" / "training" / "label_02"
OUTPUT_BASE = ROOT / "detectors" / "gt"
SCORE_LOGIT = 10.0  # logistic^{-1}(~0.99995)
KEEP_TYPES = {"Car"}


def ensure_dir(path: Path) -> None:
    path.mkdir(parents=True, exist_ok=True)


def process_sequence(label_file: Path) -> None:
    seq = label_file.stem
    target_dir = OUTPUT_BASE / "training" / seq
    ensure_dir(target_dir)

    frames = {}
    with label_file.open("r") as f:
        for line in f:
            parts = line.strip().split()
            if not parts:
                continue
            frame = int(parts[0])
            obj_type = parts[2]
            if obj_type not in KEEP_TYPES:
                continue
            truncated, occluded, alpha = parts[3:6]
            left, top, right, bottom = parts[6:10]
            height, width, length = parts[10:13]
            x, y, z, rot = parts[13:17]
            formatted = " ".join([
                obj_type.lower(),
                truncated,
                occluded,
                alpha,
                left,
                top,
                right,
                bottom,
                height,
                width,
                length,
                x,
                y,
                z,
                rot,
                f"{SCORE_LOGIT:.6f}",
            ])
            frames.setdefault(frame, []).append(formatted)

    for frame_id, entries in frames.items():
        frame_path = target_dir / f"{frame_id:06d}.txt"
        with frame_path.open("w") as out:
            out.write("\n".join(entries))


def main() -> None:
    ensure_dir(OUTPUT_BASE / "training")
    for label_file in sorted(LABEL_DIR.glob("*.txt")):
        process_sequence(label_file)
        print(f"已转换 {label_file.name}")


if __name__ == "__main__":
    main()
