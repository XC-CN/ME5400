#!/bin/bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
ROOT_DIR=$(cd "$SCRIPT_DIR/../.." && pwd)
CONFIG="$SCRIPT_DIR/ME5400.rviz"

if [[ ! -f "$CONFIG" ]]; then
  echo "[错误] 找不到 RViz 配置: $CONFIG" >&2
  exit 1
fi

log() {
  echo "[RViz] $*" >&2
}

find_target_monitor_geometry() {
  local target="${RVIZ_TARGET_MONITOR:-secondary}"
  local line index flags geometry name

  while IFS= read -r line; do
    [[ "$line" =~ ^[[:space:]]*([0-9]+):[[:space:]]+([^[:space:]]+)[[:space:]]+([0-9]+/[0-9]+x[0-9]+/[0-9]+\+[0-9]+\+[0-9]+)[[:space:]]+(.+)$ ]] || continue
    index="${BASH_REMATCH[1]}"
    flags="${BASH_REMATCH[2]}"
    geometry="${BASH_REMATCH[3]}"
    name="${BASH_REMATCH[4]}"

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
  done < <(xrandr --listactivemonitors 2>/dev/null | tail -n +2)

  return 1
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

find_window_id_by_pid() {
  local pid="$1"

  wmctrl -lp 2>/dev/null | awk -v pid="$pid" '$3 == pid { print $1; exit }' || true
}

place_rviz_window() {
  local target_pid="$1"
  local target_monitor="${RVIZ_TARGET_MONITOR:-secondary}"
  local wait_steps="${RVIZ_WINDOW_WAIT_STEPS:-80}"
  local i
  local geometry width height pos_x pos_y window_id=""

  if [[ "${RVIZ_ENABLE_WINDOW_PLACEMENT:-1}" != "1" ]]; then
    return 0
  fi

  if [[ -z "${DISPLAY:-}" ]]; then
    log "未检测到 DISPLAY，跳过窗口定位"
    return 0
  fi

  if ! command -v xrandr >/dev/null 2>&1 || ! command -v wmctrl >/dev/null 2>&1; then
    log "缺少 xrandr 或 wmctrl，跳过窗口定位"
    return 0
  fi

  geometry=$(find_target_monitor_geometry) || {
    log "未找到目标显示器 (${target_monitor})，RViz 保持默认位置"
    return 0
  }

  read -r width height pos_x pos_y < <(parse_monitor_geometry "$geometry") || {
    log "无法解析显示器几何信息: $geometry"
    return 0
  }

  for ((i = 0; i < wait_steps; i++)); do
    if ! kill -0 "$target_pid" 2>/dev/null; then
      return 0
    fi

    window_id=$(find_window_id_by_pid "$target_pid")
    if [[ -n "$window_id" ]]; then
      break
    fi

    sleep 0.25
  done

  if [[ -z "$window_id" ]]; then
    log "未在等待时间内找到 RViz 窗口，保持默认位置"
    return 0
  fi

  wmctrl -i -r "$window_id" -b remove,fullscreen,maximized_vert,maximized_horz >/dev/null 2>&1 || true
  sleep 0.1
  wmctrl -i -r "$window_id" -e "0,${pos_x},${pos_y},${width},${height}" >/dev/null 2>&1 || true
  wmctrl -i -a "$window_id" >/dev/null 2>&1 || true
  sleep 0.1
  wmctrl -i -r "$window_id" -b add,fullscreen >/dev/null 2>&1 || true
}

source "$ROOT_DIR/Scripts/utils/setup_ros_env.sh"

# 保持当前脚本 PID 不变，便于上层脚本继续按原方式管理 RViz 进程。
place_rviz_window "$$" &
exec rosrun rviz rviz -d "$CONFIG"
