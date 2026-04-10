#!/bin/bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PROJECT_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
SEQ_ID="0020"
CHECK_ONLY=false
SKIP_BUILD=false

usage() {
  cat <<'EOF'
Usage: bash agent/init.sh [--seq <id>] [--check-only] [--skip-build]

Purpose:
  Rehydrate a fresh agent session with the minimum context and environment checks
  needed to work on this repository safely.

Options:
  --seq <id>      Sequence to inspect (default: 0020)
  --check-only    Do not build catkin_ws; only validate the environment
  --skip-build    Skip catkin build even in normal mode
  -h, --help      Show this message
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --seq)
      SEQ_ID="$2"
      shift 2
      ;;
    --check-only)
      CHECK_ONLY=true
      shift
      ;;
    --skip-build)
      SKIP_BUILD=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[ERROR] Unknown argument: $1" >&2
      usage >&2
      exit 1
      ;;
  esac
done

require_path() {
  local path="$1"
  local label="$2"
  if [[ ! -e "$path" ]]; then
    echo "[ERROR] Missing $label: $path" >&2
    exit 1
  fi
  echo "[OK] $label: $path"
}

echo "[INFO] Project root: $PROJECT_ROOT"
require_path "$PROJECT_ROOT/.git" "git repository"
require_path "$PROJECT_ROOT/agent/feature_list.json" "agent feature list"
require_path "$PROJECT_ROOT/agent/progress.md" "agent progress log"
require_path "$PROJECT_ROOT/Scripts/run_all.sh" "dual-run entrypoint"
require_path "$PROJECT_ROOT/Data_Tracking/rosbags/seq_${SEQ_ID}_nodet.bag" "rosbag"
require_path "$PROJECT_ROOT/Data_Tracking/training/oxts/${SEQ_ID}.txt" "ground-truth file"

if [[ ! -f /opt/ros/noetic/setup.bash ]]; then
  echo "[ERROR] ROS Noetic setup not found at /opt/ros/noetic/setup.bash" >&2
  exit 1
fi

set +u
source "$PROJECT_ROOT/Scripts/utils/setup_ros_env.sh"
set -u
echo "[OK] ROS environment loaded"

if [[ -f "$HOME/miniconda3/etc/profile.d/conda.sh" ]]; then
  # Some conda activate hooks are not nounset-safe, so relax `set -u` only
  # around activation and restore strict mode immediately afterwards.
  set +u
  # shellcheck disable=SC1091
  source "$HOME/miniconda3/etc/profile.d/conda.sh"
  if conda env list | awk '{print $1}' | grep -qx "ME5400"; then
    conda activate ME5400
    echo "[OK] Conda environment activated: ME5400"
  else
    echo "[WARN] Conda env ME5400 not found; continuing without activation"
  fi
  set -u
else
  echo "[WARN] miniconda profile not found; continuing without conda activation"
fi

echo "[INFO] Python: $(python3 --version 2>&1)"
echo "[INFO] Git branch: $(git -C "$PROJECT_ROOT" branch --show-current 2>/dev/null || echo detached)"

if [[ "$CHECK_ONLY" == "false" && "$SKIP_BUILD" == "false" ]]; then
  echo "[INFO] Building catkin workspace"
  "$PROJECT_ROOT/Scripts/utils/build_catkin_ws.sh"
else
  echo "[INFO] Skipping catkin build"
fi

echo "[INFO] Recommended next steps:"
echo "  1. Read agent/progress.md"
echo "  2. Read agent/feature_list.json"
echo "  3. Review recent git log"
echo "  4. Run bash agent/check_smoke.sh --seq $SEQ_ID"
