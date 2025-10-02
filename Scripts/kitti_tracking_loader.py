#!/usr/bin/env python3
"""Utilities to fetch KITTI Tracking frames (point cloud, IMU, detections, calib)."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np


@dataclass
class Detection:
    frame: int
    track_id: int
    cls: str
    score: float
    bbox: np.ndarray  # x1,y1,x2,y2
    dimensions: np.ndarray  # h,w,l
    location: np.ndarray  # x,y,z in camera coord
    rotation_y: float

    def to_dict(self) -> Dict:
        return {
            "class": self.cls,
            "bbox": self.bbox.copy(),
            "score": self.score,
            "dimensions": self.dimensions.copy(),
            "location": self.location.copy(),
            "rotation_y": self.rotation_y,
        }


class KITTITrackingLoader:
    def __init__(self, root: Path, det_root: Optional[Path] = None) -> None:
        self.root = Path(root)
        self.seq_root = self.root / "sequences"
        self.oxts_root = self.root / "oxts"
        self.det_root = Path(det_root) if det_root is not None else self.root / "det_tracking_lsvm"
        self._oxts_cache: Dict[str, np.ndarray] = {}
        self._det_cache: Dict[str, List[Detection]] = {}
        self._time_cache: Dict[str, np.ndarray] = {}
        self._calib_cache: Dict[str, Dict[str, np.ndarray]] = {}

    def _velodyne_file(self, seq: str, frame: int) -> Path:
        return self.seq_root / seq / "velodyne" / f"{frame:06d}.bin"

    def _oxts_dir(self, seq: str) -> Path:
        # tracking/oxts/{seq}/[data]/
        base = self.oxts_root / seq
        if (base / "data").is_dir():
            return base / "data"
        return base

    def _load_oxts(self, seq: str) -> np.ndarray:
        if seq in self._oxts_cache:
            return self._oxts_cache[seq]
        oxts_dir = self._oxts_dir(seq)
        files = sorted(oxts_dir.glob("*.txt"))
        if not files:
            raise FileNotFoundError(f"No OXTS files for sequence {seq} in {oxts_dir}")
        data = []
        for fp in files:
            vals = [float(x) for x in fp.read_text().strip().split()]
            data.append(vals)
        arr = np.asarray(data, dtype=np.float64)
        self._oxts_cache[seq] = arr
        return arr

    def _load_times(self, seq: str) -> Optional[np.ndarray]:
        if seq in self._time_cache:
            return self._time_cache[seq]
        time_file = self.seq_root / seq / "times.txt"
        if not time_file.exists():
            return None
        times = []
        for line in time_file.read_text().strip().splitlines():
            try:
                times.append(float(line))
            except ValueError:
                continue
        arr = np.asarray(times, dtype=np.float64)
        self._time_cache[seq] = arr
        return arr

    def _load_calib(self, seq: str) -> Dict[str, np.ndarray]:
        if seq in self._calib_cache:
            return self._calib_cache[seq]
        calib_path = self.seq_root / seq / "calib.txt"
        calib: Dict[str, np.ndarray] = {}
        with calib_path.open("r") as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                key, values = line.split(":", 1)
                arr = np.fromstring(values, sep=" ")
                calib[key] = arr
        self._calib_cache[seq] = calib
        return calib

    def _load_detections(self, seq: str) -> List[Detection]:
        if seq in self._det_cache:
            return self._det_cache[seq]
        det_file = self.det_root / f"{seq}.txt"
        if not det_file.exists():
            raise FileNotFoundError(f"Detection file not found: {det_file}")
        dets: List[Detection] = []
        with det_file.open("r") as f:
            for line in f:
                parts = line.strip().split()
                if not parts:
                    continue
                frame = int(parts[0])
                track = int(parts[1])
                cls = parts[2]
                truncated = float(parts[3])  # kept in case needed later
                occluded = float(parts[4])
                alpha = float(parts[5])
                bbox = np.array(list(map(float, parts[6:10])), dtype=np.float64)
                dims = np.array(list(map(float, parts[10:13])), dtype=np.float64)  # h, w, l
                loc = np.array(list(map(float, parts[13:16])), dtype=np.float64)
                rot_y = float(parts[16])
                score = float(parts[17]) if len(parts) > 17 else 1.0
                dets.append(
                    Detection(
                        frame=frame,
                        track_id=track,
                        cls=cls,
                        score=score,
                        bbox=bbox,
                        dimensions=dims,
                        location=loc,
                        rotation_y=rot_y,
                    )
                )
        self._det_cache[seq] = dets
        return dets

    def get_pointcloud(self, seq: str, frame: int) -> np.ndarray:
        pc_file = self._velodyne_file(seq, frame)
        raw = np.fromfile(pc_file, dtype=np.float32)
        if raw.size % 4 != 0:
            raise ValueError(f"Unexpected velodyne format: {pc_file}")
        return raw.reshape(-1, 4)

    def get_imu(self, seq: str, frame: int) -> Dict[str, np.ndarray]:
        oxts = self._load_oxts(seq)
        if frame < oxts.shape[0]:
            data = oxts[frame]
        else:
            idx = min(frame, oxts.shape[0] - 1)
            data = oxts[idx]
        imu_keys = [
            "lat",
            "lon",
            "alt",
            "roll",
            "pitch",
            "yaw",
            "vn",
            "ve",
            "vf",
            "vl",
            "vu",
            "ax",
            "ay",
            "az",
            "af",
            "al",
            "au",
            "wx",
            "wy",
            "wz",
            "wf",
            "wl",
            "wu",
            "pos_accuracy",
            "vel_accuracy",
            "navstat",
            "numsats",
            "posmode",
            "velmode",
            "orimode",
        ]
        imu = {key: data[i] for i, key in enumerate(imu_keys)}
        imu["raw"] = data
        return imu

    def get_timestamp(self, seq: str, frame: int) -> Optional[float]:
        times = self._load_times(seq)
        if times is None:
            return None
        if frame < len(times):
            return float(times[frame])
        return float(times[-1])

    def get_detections(self, seq: str, frame: int) -> List[Dict]:
        dets = self._load_detections(seq)
        return [d.to_dict() for d in dets if d.frame == frame]

    def get_calib(self, seq: str) -> Dict[str, np.ndarray]:
        return self._load_calib(seq)

    def get_frame(self, seq_id: str, frame_id: int) -> Dict:
        seq = seq_id.zfill(4) if len(seq_id) < 4 else seq_id
        pointcloud = self.get_pointcloud(seq, frame_id)
        imu = self.get_imu(seq, frame_id)
        dets = self.get_detections(seq, frame_id)
        timestamp = self.get_timestamp(seq, frame_id)
        calib = self.get_calib(seq)
        return {
            "pointcloud": pointcloud,
            "imu": imu,
            "detections": dets,
            "timestamp": timestamp,
            "calib": calib,
        }


def get_frame(seq_id: str, frame_id: int,
              dataset_root: str = "tracking",
              detector_root: Optional[str] = None) -> Dict:
    loader = KITTITrackingLoader(Path(dataset_root),
                                 Path(detector_root) if detector_root else None)
    return loader.get_frame(seq_id, frame_id)


__all__ = ["KITTITrackingLoader", "get_frame"]
