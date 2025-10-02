#!/usr/bin/env python3
"""Run RViz headless and verify visualization pipeline."""
from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path
from typing import List, Set

import yaml


def parse_rviz_topics(config_path: Path) -> Set[str]:
    try:
        with config_path.open("r") as f:
            data = yaml.safe_load(f)
    except Exception:
        return set()

    topics: Set[str] = set()

    def recurse(obj):
        if isinstance(obj, dict):
            for key, value in obj.items():
                if isinstance(key, str) and key.lower() == "topic" and isinstance(value, str):
                    if value.startswith("/"):
                        topics.add(value)
                recurse(value)
        elif isinstance(obj, list):
            for item in obj:
                recurse(item)

    recurse(data)
    return topics


def rostopic_list() -> List[str]:
    try:
        output = subprocess.check_output(["rostopic", "list"], text=True, timeout=5)
        return [line.strip() for line in output.splitlines() if line.strip()]
    except Exception:
        return []


def run_rviz(config: Path) -> subprocess.Popen:
    cmd = [
        "xvfb-run",
        "-s",
        "-screen 0 1280x1024x24",
        "rviz",
        "-d",
        str(config),
    ]
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    return proc


def capture_screenshot(output: Path) -> bool:
    try:
        subprocess.check_call(
            ["rosrun", "rviz", "rviz_screenshot", "--output", str(output)], timeout=10
        )
    except Exception:
        return False
    return output.exists() and output.stat().st_size > 0


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("config", type=Path)
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--screenshot", type=Path, default=Path("screenshot.png"))
    args = parser.parse_args()

    config_path = args.config
    topics_expected = parse_rviz_topics(config_path)

    proc = run_rviz(config_path)
    time.sleep(args.timeout)

    rviz_running = proc.poll() is None
    stdout_data = ""
    stderr_data = ""
    log_status = "ok"

    if not rviz_running:
        stdout_data, stderr_data = proc.communicate(timeout=1)
        log_status = "error"
    else:
        # Non-blocking read of available data
        try:
            stdout_data = proc.stdout.read() if proc.stdout else ""
        except Exception:
            stdout_data = ""
        try:
            stderr_data = proc.stderr.read() if proc.stderr else ""
        except Exception:
            stderr_data = ""

    error_keywords = ["[ERROR]", "Segmentation fault", "QXcbConnection"]
    if any(keyword in stderr_data for keyword in error_keywords):
        log_status = "error"

    topics_available = rostopic_list()
    topics_found = [topic for topic in topics_expected if topic in topics_available]
    topics_missing = [topic for topic in topics_expected if topic not in topics_available]

    screenshot_ok = False
    screenshot_path = args.screenshot
    if rviz_running:
        screenshot_ok = capture_screenshot(screenshot_path)

    if rviz_running:
        proc.send_signal(signal.SIGTERM)
        try:
            proc.wait(timeout=2)
        except subprocess.TimeoutExpired:
            proc.kill()
    else:
        proc.wait(timeout=1)

    result = {
        "rviz_process": "running" if rviz_running else "failed",
        "log_status": log_status,
        "topics_verified": topics_found,
        "topics_missing": topics_missing,
        "screenshot": {
            "path": str(screenshot_path),
            "exists": screenshot_path.exists(),
            "size_kb": round(screenshot_path.stat().st_size / 1024.0, 2)
            if screenshot_path.exists()
            else 0.0,
            "success": screenshot_ok,
        },
        "conclusion": "rviz可视化正常"
        if rviz_running and log_status == "ok" and screenshot_ok
        else "rviz可视化异常",
    }

    print(json.dumps(result, ensure_ascii=False, indent=2))

    # cleanup
    if screenshot_path.exists() and not screenshot_ok:
        try:
            screenshot_path.unlink()
        except OSError:
            pass


if __name__ == "__main__":
    main()
