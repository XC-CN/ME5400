#!/usr/bin/env python3
"""根据 KITTI tracking 数据集中的 oxts 数据生成 pose 矩阵文件。"""
from pathlib import Path
from typing import Dict
import numpy as np
from pykitti import utils

ROOT = Path(__file__).resolve().parents[1] / "MCTrack" / "data" / "kitti" / "datasets"


def read_calib_mat(calib_path: Path) -> Dict[str, np.ndarray]:
    data: Dict[str, np.ndarray] = {}
    with calib_path.open("r") as f:
        for raw_line in f:
            line = raw_line.strip()
            if not line:
                continue
            if ":" in line:
                key, value = line.split(":", 1)
            else:
                parts = line.split()
                key, value = parts[0], " ".join(parts[1:])
            numbers = [float(x) for x in value.strip().split()]
            data[key.strip()] = np.array(numbers, dtype=np.float64)
    return data


def to_homogeneous(mat: np.ndarray) -> np.ndarray:
    mat = mat.reshape(3, 4)
    bottom = np.array([[0.0, 0.0, 0.0, 1.0]])
    return np.vstack((mat, bottom))


def generate_pose_for_split(split: str) -> None:
    split_dir = ROOT / split
    calib_dir = split_dir / "calib"
    oxts_dir = split_dir / "oxts"
    pose_dir = split_dir / "pose"
    pose_dir.mkdir(parents=True, exist_ok=True)

    for oxts_file in sorted(oxts_dir.glob("*.txt")):
        seq = oxts_file.stem
        calib_file = calib_dir / f"{seq}.txt"
        if not calib_file.exists():
            raise FileNotFoundError(f"未找到标定文件: {calib_file}")

        calib = read_calib_mat(calib_file)
        if "Tr_imu_velo" not in calib:
            raise KeyError(f"标定文件缺少 Tr_imu_velo: {calib_file}")

        T_imu_velo = to_homogeneous(calib["Tr_imu_velo"])

        oxts_packets = utils.load_oxts_packets_and_poses([str(oxts_file)])
        pose_path = pose_dir / f"{seq}.txt"
        with pose_path.open("w") as pose_f:
            for packet in oxts_packets:
                T_w_imu = packet.T_w_imu
                T_w_velo = T_w_imu @ T_imu_velo
                line = " ".join(f"{num:.12e}" for num in T_w_velo[:3].reshape(-1))
                pose_f.write(line + "\n")
        print(f"生成 {pose_path.relative_to(ROOT)}")


def main() -> None:
    for split in ("training", "testing"):
        generate_pose_for_split(split)


if __name__ == "__main__":
    main()
