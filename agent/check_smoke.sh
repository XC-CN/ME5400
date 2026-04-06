#!/bin/bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PROJECT_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
SEQ_ID="0020"
FULL_RUN=false

usage() {
  cat <<'EOF'
Usage: bash agent/check_smoke.sh [--seq <id>] [--full]

Modes:
  quick (default): re-evaluate saved trajectories and verify metrics.txt/metrics.json
  --full          : run the full headless dual pipeline, then validate metrics outputs
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --seq)
      SEQ_ID="$2"
      shift 2
      ;;
    --full)
      FULL_RUN=true
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

RESULT_DIR="$PROJECT_ROOT/Results/${SEQ_ID}_results"
GT_FILE="$PROJECT_ROOT/Data_Tracking/training/oxts/${SEQ_ID}.txt"

validate_metrics() {
  local metrics_json="$1"
  python3 - "$metrics_json" <<'PY'
import json
import pathlib
import sys

metrics_path = pathlib.Path(sys.argv[1])
if not metrics_path.exists():
    raise SystemExit(f"missing metrics json: {metrics_path}")

payload = json.loads(metrics_path.read_text())
required = ["sequence", "optimized", "artifacts"]
for key in required:
    if key not in payload:
        raise SystemExit(f"missing key: {key}")

opt = payload["optimized"]
for key in ["rmse", "mean_error", "max_error", "num_samples"]:
    if key not in opt:
        raise SystemExit(f"missing optimized key: {key}")

artifacts = payload["artifacts"]
for key in ["metrics_txt", "metrics_json", "plot"]:
    path = pathlib.Path(artifacts[key])
    if not path.exists():
        raise SystemExit(f"missing artifact {key}: {path}")

print(
    json.dumps(
        {
            "sequence": payload["sequence"],
            "optimized_rmse": opt["rmse"],
            "improvement_percent": payload.get("improvement_percent"),
            "metrics_json": str(metrics_path),
        },
        ensure_ascii=True,
    )
)
PY
}

echo "[INFO] Initializing environment"
bash "$SCRIPT_DIR/init.sh" --check-only --seq "$SEQ_ID"

if [[ "$FULL_RUN" == "true" ]]; then
  echo "[INFO] Running full headless validation for sequence $SEQ_ID"
  bash "$PROJECT_ROOT/Scripts/run_all.sh" --headless "$SEQ_ID"
  METRICS_JSON="$RESULT_DIR/metrics.json"
else
  echo "[INFO] Running quick smoke on cached artifacts for sequence $SEQ_ID"
  PRED_FILE="$RESULT_DIR/trajectory.txt"
  BASELINE_FILE="$RESULT_DIR/trajectory_baseline.txt"
  TMP_OUTPUT_ROOT="${TMPDIR:-/tmp}/me5400_agent_smoke"

  for path in "$PRED_FILE" "$BASELINE_FILE" "$GT_FILE"; do
    if [[ ! -f "$path" ]]; then
      echo "[ERROR] Missing required file: $path" >&2
      exit 1
    fi
  done

  export MPLCONFIGDIR="${TMPDIR:-/tmp}/me5400_mplconfig"
  mkdir -p "$MPLCONFIGDIR"

  python3 "$PROJECT_ROOT/Scripts/evaluation/evaluate_trajectory.py" \
    --pred "$PRED_FILE" \
    --baseline "$BASELINE_FILE" \
    --gt "$GT_FILE" \
    --output "$TMP_OUTPUT_ROOT"

  METRICS_JSON="$TMP_OUTPUT_ROOT/${SEQ_ID}_results/metrics.json"
fi

echo "[INFO] Validating metrics payload"
validate_metrics "$METRICS_JSON"
echo "[INFO] Smoke check passed"
