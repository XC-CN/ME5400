#!/bin/bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
ROOT_DIR=$(cd "$SCRIPT_DIR/../.." && pwd)
CONFIG="$SCRIPT_DIR/ME5400.rviz"

if [[ ! -f "$CONFIG" ]]; then
  echo "[错误] 找不到 RViz 配置: $CONFIG" >&2
  exit 1
fi

debug_log() {
  if [[ "${RVIZ_WINDOW_DEBUG:-0}" == "1" ]]; then
    echo "[RViz] $*" >&2
  fi
}

cleanup_runtime_config() {
  if [[ -n "${RUNTIME_CONFIG:-}" && "$RUNTIME_CONFIG" != "$CONFIG" && -f "$RUNTIME_CONFIG" ]]; then
    rm -f "$RUNTIME_CONFIG"
  fi
}

list_monitors() {
  xrandr --listactivemonitors 2>/dev/null | tail -n +2
}

get_monitor_count() {
  local count
  count=$(list_monitors | sed '/^[[:space:]]*$/d' | wc -l)
  echo "${count//[[:space:]]/}"
}

parse_monitor_line() {
  local line="$1"

  [[ "$line" =~ ^[[:space:]]*([0-9]+):[[:space:]]+([^[:space:]]+)[[:space:]]+([0-9]+/[0-9]+x[0-9]+/[0-9]+\+[0-9]+\+[0-9]+)[[:space:]]+(.+)$ ]] || return 1
  printf '%s|%s|%s|%s\n' \
    "${BASH_REMATCH[1]}" \
    "${BASH_REMATCH[2]}" \
    "${BASH_REMATCH[3]}" \
    "${BASH_REMATCH[4]}"
}

parse_monitor_geometry() {
  local geometry="$1"

  [[ "$geometry" =~ ^([0-9]+)/[0-9]+x([0-9]+)/[0-9]+\+([0-9]+)\+([0-9]+)$ ]] || return 1
  printf '%s %s %s %s\n' \
    "${BASH_REMATCH[1]}" \
    "${BASH_REMATCH[2]}" \
    "${BASH_REMATCH[3]}" \
    "${BASH_REMATCH[4]}"
}

monitor_contains_point() {
  local geometry="$1"
  local point_x="$2"
  local point_y="$3"
  local width height pos_x pos_y

  read -r width height pos_x pos_y < <(parse_monitor_geometry "$geometry") || return 1

  if (( point_x >= pos_x && point_x < pos_x + width && point_y >= pos_y && point_y < pos_y + height )); then
    return 0
  fi

  return 1
}

get_active_window_center() {
  local active_id pos_x pos_y width height

  active_id=$(xprop -root _NET_ACTIVE_WINDOW 2>/dev/null | awk '/_NET_ACTIVE_WINDOW/ {print $NF}') || true
  if [[ -z "$active_id" || "$active_id" == "0x0" ]]; then
    return 1
  fi

  read -r pos_x pos_y width height < <(
    xwininfo -id "$active_id" 2>/dev/null | awk '
      /Absolute upper-left X:/ {x = $4}
      /Absolute upper-left Y:/ {y = $4}
      /^  Width:/ {w = $2}
      /^  Height:/ {h = $2}
      END {
        if (x == "" || y == "" || w == "" || h == "") {
          exit 1
        }
        printf "%d %d %d %d\n", x, y, w, h
      }
    '
  ) || return 1

  printf '%s %s\n' "$((pos_x + width / 2))" "$((pos_y + height / 2))"
}

find_target_monitor_geometry() {
  local requested_target="${RVIZ_TARGET_MONITOR:-secondary}"
  local target="$requested_target"
  local count line parsed index flags geometry name active_x active_y
  local lowered_target

  count=$(get_monitor_count)
  if [[ -z "$count" || "$count" -lt 2 ]]; then
    return 1
  fi

  lowered_target=$(printf '%s' "$target" | tr '[:upper:]' '[:lower:]')
  if [[ "$lowered_target" == "skydata" ]]; then
    target="secondary"
    lowered_target="secondary"
  fi

  if [[ "$target" == "other" ]]; then
    if read -r active_x active_y < <(get_active_window_center); then
      while IFS= read -r line; do
        parsed=$(parse_monitor_line "$line") || continue
        IFS='|' read -r index flags geometry name <<< "$parsed"
        if ! monitor_contains_point "$geometry" "$active_x" "$active_y"; then
          printf '%s\n' "$geometry"
          return 0
        fi
      done < <(list_monitors)
    fi

    target="secondary"
  fi

  while IFS= read -r line; do
    parsed=$(parse_monitor_line "$line") || continue
    IFS='|' read -r index flags geometry name <<< "$parsed"

    if [[ "$target" == "secondary" ]]; then
      if [[ "$flags" != *"*"* ]]; then
        printf '%s\n' "$geometry"
        return 0
      fi
      continue
    fi

    if [[ "$target" =~ ^[0-9]+$ ]] && [[ "$index" == "$target" ]]; then
      printf '%s\n' "$geometry"
      return 0
    fi

    if [[ "$name" == "$target" ]]; then
      printf '%s\n' "$geometry"
      return 0
    fi
  done < <(list_monitors)

  return 1
}

prepare_runtime_config() {
  local source_config="$1"
  local geometry="$2"
  local width height pos_x pos_y tmp_config

  if [[ -z "$geometry" ]]; then
    printf '%s\n' "$source_config"
    return 0
  fi

  read -r width height pos_x pos_y < <(parse_monitor_geometry "$geometry") || {
    printf '%s\n' "$source_config"
    return 0
  }

  tmp_config=$(mktemp /tmp/me5400_rviz.XXXXXX.rviz)
  awk \
    -v width="$width" \
    -v height="$height" \
    -v pos_x="$pos_x" \
    -v pos_y="$pos_y" \
    '
      BEGIN { in_window = 0 }
      /^Window Geometry:/ { in_window = 1; print; next }
      in_window && /^[^[:space:]]/ { in_window = 0 }
      {
        if (in_window) {
          sub(/Height: [0-9]+/, "Height: " height)
          sub(/Width: [0-9]+/, "Width: " width)
          sub(/X: [0-9]+/, "X: " pos_x)
          sub(/Y: [0-9]+/, "Y: " pos_y)
        }
        print
      }
    ' "$source_config" > "$tmp_config"

  printf '%s\n' "$tmp_config"
}

list_rviz_window_ids() {
  wmctrl -lx 2>/dev/null | awk '$3 == "rviz.rviz" { print $1 }' || true
}

find_window_id_by_pid() {
  local pid="$1"

  wmctrl -lp 2>/dev/null | awk -v pid="$pid" '$3 == pid { print $1; exit }' || true
}

find_new_rviz_window_id() {
  local initial_windows="$1"
  local window_id

  while IFS= read -r window_id; do
    [[ -z "$window_id" ]] && continue
    if ! grep -Fxq "$window_id" <<< "$initial_windows"; then
      printf '%s\n' "$window_id"
      return 0
    fi
  done < <(list_rviz_window_ids)

  return 1
}

find_rviz_window_id() {
  local target_pid="$1"
  local initial_windows="$2"
  local window_id

  window_id=$(find_window_id_by_pid "$target_pid")
  if [[ -n "$window_id" ]]; then
    printf '%s\n' "$window_id"
    return 0
  fi

  window_id=$(find_new_rviz_window_id "$initial_windows")
  if [[ -n "$window_id" ]]; then
    printf '%s\n' "$window_id"
    return 0
  fi

  return 1
}

get_window_geometry() {
  local window_id="$1"

  wmctrl -lG 2>/dev/null | awk -v wid="$window_id" '$1 == wid { printf "%s %s %s %s\n", $3, $4, $5, $6; exit }' || true
}

window_is_on_target_monitor() {
  local window_id="$1"
  local width="$2"
  local height="$3"
  local pos_x="$4"
  local pos_y="$5"
  local win_x win_y win_w win_h center_x center_y

  read -r win_x win_y win_w win_h < <(get_window_geometry "$window_id") || return 1
  if [[ -z "${win_x:-}" || -z "${win_y:-}" || -z "${win_w:-}" || -z "${win_h:-}" ]]; then
    return 1
  fi

  center_x=$((win_x + win_w / 2))
  center_y=$((win_y + win_h / 2))

  if (( center_x >= pos_x && center_x < pos_x + width && center_y >= pos_y && center_y < pos_y + height )); then
    return 0
  fi

  return 1
}

move_window_to_monitor() {
  local window_id="$1"
  local width="$2"
  local height="$3"
  local pos_x="$4"
  local pos_y="$5"
  local probe_w probe_h anchor_x anchor_y

  probe_w=$(( width > 1600 ? 1600 : width - 80 ))
  probe_h=$(( height > 1200 ? 1200 : height - 80 ))
  (( probe_w < 640 )) && probe_w=640
  (( probe_h < 480 )) && probe_h=480

  anchor_x=$(( pos_x + 40 ))
  anchor_y=$(( pos_y + 40 ))

  debug_log "尝试移动 RViz 窗口 $window_id 到副屏坐标 ${pos_x},${pos_y} (${width}x${height})"

  wmctrl -i -r "$window_id" -b remove,fullscreen,maximized_vert,maximized_horz >/dev/null 2>&1 || true
  sleep 0.1
  wmctrl -i -r "$window_id" -e "0,${anchor_x},${anchor_y},${probe_w},${probe_h}" >/dev/null 2>&1 || true
  wmctrl -i -a "$window_id" >/dev/null 2>&1 || true
  sleep 0.15
  wmctrl -i -r "$window_id" -b add,maximized_vert,maximized_horz >/dev/null 2>&1 || true
  sleep 0.2

  if window_is_on_target_monitor "$window_id" "$width" "$height" "$pos_x" "$pos_y"; then
    debug_log "RViz 窗口 $window_id 已切到副屏并最大化"
    return 0
  fi

  wmctrl -i -r "$window_id" -b remove,maximized_vert,maximized_horz >/dev/null 2>&1 || true
  sleep 0.1
  wmctrl -i -r "$window_id" -e "0,${anchor_x},${anchor_y},${probe_w},${probe_h}" >/dev/null 2>&1 || true
  wmctrl -i -a "$window_id" >/dev/null 2>&1 || true
  sleep 0.1
  wmctrl -i -r "$window_id" -b add,maximized_vert,maximized_horz >/dev/null 2>&1 || true
  debug_log "RViz 窗口 $window_id 已切到副屏并最大化"
}

place_rviz_window() {
  local target_pid="$1"
  local initial_windows="$2"
  local wait_steps="${RVIZ_WINDOW_WAIT_STEPS:-80}"
  local i
  local geometry width height pos_x pos_y window_id=""

  if [[ "${RVIZ_ENABLE_WINDOW_PLACEMENT:-1}" != "1" ]]; then
    return 0
  fi

  if [[ -z "${DISPLAY:-}" ]]; then
    return 0
  fi

  if ! command -v xrandr >/dev/null 2>&1 || ! command -v wmctrl >/dev/null 2>&1 || ! command -v xprop >/dev/null 2>&1 || ! command -v xwininfo >/dev/null 2>&1; then
    return 0
  fi

  geometry=$(find_target_monitor_geometry) || return 0

  read -r width height pos_x pos_y < <(parse_monitor_geometry "$geometry") || return 0

  for ((i = 0; i < wait_steps; i++)); do
    window_id=$(find_rviz_window_id "$target_pid" "$initial_windows" || true)
    if [[ -n "$window_id" ]]; then
      debug_log "检测到 RViz 窗口 $window_id，开始切到副屏"
      move_window_to_monitor "$window_id" "$width" "$height" "$pos_x" "$pos_y"
      return 0
    fi

    if ! kill -0 "$target_pid" 2>/dev/null && [[ -z "$(list_rviz_window_ids)" ]]; then
      return 0
    fi

    sleep 0.25
  done

  debug_log "未在等待时间内找到 RViz 窗口，保持默认位置"
  return 0
}

source "$ROOT_DIR/Scripts/utils/setup_ros_env.sh"

TARGET_GEOMETRY=$(find_target_monitor_geometry || true)
RUNTIME_CONFIG=$(prepare_runtime_config "$CONFIG" "${TARGET_GEOMETRY:-}")
INITIAL_RVIZ_WINDOWS=$(list_rviz_window_ids)
rosrun rviz rviz -d "$RUNTIME_CONFIG" &
RVIZ_PID=$!

forward_shutdown() {
  if [[ -n "${RVIZ_PID:-}" ]] && kill -0 "$RVIZ_PID" 2>/dev/null; then
    kill -TERM "$RVIZ_PID" 2>/dev/null || true
  fi
  cleanup_runtime_config
}

trap forward_shutdown INT TERM
trap cleanup_runtime_config EXIT

place_rviz_window "$RVIZ_PID" "$INITIAL_RVIZ_WINDOWS"

wait "$RVIZ_PID"
RVIZ_STATUS=$?

exit "$RVIZ_STATUS"
