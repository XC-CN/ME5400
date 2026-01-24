#!/bin/bash
set -e

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
ROOT_DIR=$(cd "$SCRIPT_DIR/../.." && pwd)
CATKIN_WS="$ROOT_DIR/catkin_ws"

usage() {
  cat <<'EOF'
用法: run_fastlio.sh [mapping|pose_bridge|both] [参数...]

  mapping       启动 fast_lio 的 mapping_velodyne.launch
  pose_bridge   启动 fastlio_pose_bridge.py
  both          同时启动 mapping 与 pose_bridge（mapping 在后台运行）
  -h, --help    打印本帮助信息

示例:
  ./run_fastlio.sh mapping rviz:=false
  ./run_fastlio.sh pose_bridge
  ./run_fastlio.sh both
EOF
}

if [[ $# -eq 0 ]]; then
  MODE="mapping"
else
  case "$1" in
    -h|--help|help)
      usage
      exit 0
      ;;
    *)
      MODE="$1"
      shift
      ;;
  esac
fi

if [[ ! -d "$CATKIN_WS/devel" ]]; then
  echo "[错误] 请先编译工作空间: $CATKIN_WS" >&2
  exit 1
fi

source "$CATKIN_WS/devel/setup.bash"

case "$MODE" in
  mapping|map)
    # 默认启用权重优化（保持向后兼容）
    if [[ $# -eq 0 ]]; then
      roslaunch fast_lio mapping_velodyne.launch rviz:=false use_dynamic_weights:=true
    else
      roslaunch fast_lio mapping_velodyne.launch rviz:=false "$@"
    fi
    ;;
  pose_bridge|bridge|pose)
    rosrun ME5400 fastlio_pose_bridge.py "$@"
    ;;
  both)
    # 检查是否指定了 use_dynamic_weights 参数
    USE_WEIGHTS="true"  # 默认启用权重优化
    for arg in "$@"; do
      if [[ "$arg" == *"use_dynamic_weights"* ]]; then
        if [[ "$arg" == *"false"* ]]; then
          USE_WEIGHTS="false"
        fi
      fi
    done
    roslaunch fast_lio mapping_velodyne.launch rviz:=false use_dynamic_weights:=$USE_WEIGHTS "$@" &
    MAPPING_PID=$!
    cleanup() {
      if kill -0 "$MAPPING_PID" 2>/dev/null; then
        echo "[信息] 正在优雅关闭 FAST-LIO mapping 进程，等待地图保存..."
        # 发送 SIGTERM 信号，让进程有机会保存地图
        kill -TERM "$MAPPING_PID" 2>/dev/null || true
        
        # 使用 wait 等待进程结束
        # 如果进程正确处理了 SIGTERM，它会在保存完地图后退出
        # 我们使用 timeout 机制来防止卡死 (这里使用简单的后台wait + sleep实现timeout)
        (
            wait "$MAPPING_PID" || true
        ) &
        WAIT_PID=$!
        
        # 等待进程退出，最多5秒 (加快Ctrl+C响应)
        local wait_count=0
        local max_wait=10  # 5秒 = 10 * 0.5秒
        while kill -0 "$WAIT_PID" 2>/dev/null && [ $wait_count -lt $max_wait ]; do
            sleep 0.5
            wait_count=$((wait_count + 1))
        done

        # 如果 wait 进程还在运行，说明 fast_lio 还没退出
        if kill -0 "$WAIT_PID" 2>/dev/null; then
          echo "[警告] 进程未在5秒内退出，强制终止"
          kill -KILL "$MAPPING_PID" 2>/dev/null || true
          kill "$WAIT_PID" 2>/dev/null || true # 停止 wait 子shell
        else
           echo "[信息] FAST-LIO mapping 进程已退出"
        fi
        
        # 验证地图文件是否保存成功
        verify_map_saved
      else
        # Even if process already exited, verify map was saved
        verify_map_saved
      fi
    }
    
    verify_map_saved() {
      local pcd_file="$ROOT_DIR/PCD/scans.pcd"
      if [ -f "$pcd_file" ]; then
        local file_size=$(stat -f%z "$pcd_file" 2>/dev/null || stat -c%s "$pcd_file" 2>/dev/null || echo "0")
        local file_size_mb=$((file_size / 1024 / 1024))
        
        if [ "$file_size" -gt 0 ]; then
          # Check modification time to ensure it's a new file
          local current_time=$(date +%s)
          # Try stat -c %Y (Linux) then stat -f %m (BSD/macOS)
          local file_mtime=$(stat -c %Y "$pcd_file" 2>/dev/null || stat -f %m "$pcd_file" 2>/dev/null || echo "0")
          local time_diff=$((current_time - file_mtime))
          
          # Allow up to 60 seconds delay
          if [ "$time_diff" -gt 60 ]; then
            echo "[错误] ✗ 地图文件存在，但似乎是旧文件！"
            echo "        文件修改时间: $(date -d @$file_mtime '+%Y-%m-%d %H:%M:%S')"
            echo "        当前系统时间: $(date -d @$current_time '+%Y-%m-%d %H:%M:%S')"
            echo "        时间差异: ${time_diff} 秒 (阈值: 60秒)"
            echo "        建议：检查 FAST-LIO 是否成功保存了地图，或尝试手动删除 scan.pcd 后重试"
            return 1
          fi

          echo "[成功] ✓ 地图文件已保存: $pcd_file"
          echo "[成功] ✓ 文件大小: ${file_size_mb} MB"
        else
          echo "[错误] ✗ 地图文件存在但大小为0，保存可能失败"
          return 1
        fi
      else
        echo "[警告] ⚠ 地图文件不存在: $pcd_file"
        echo "[警告] ⚠ 可能原因：1) 点云数据为空 2) 保存被中断 3) 配置中pcd_save_en=false"
      fi
    }
    trap cleanup EXIT INT TERM
    # Run pose_bridge in foreground
    # If pose_bridge exits (e.g., due to rosbag stopping), wait for mapping to finish
    rosrun ME5400 fastlio_pose_bridge.py || true
    
    # After pose_bridge exits, check if mapping is still running
    # If mapping is still running, wait for it to finish (it will save map when ros::ok() becomes false)
    if kill -0 "$MAPPING_PID" 2>/dev/null; then
      echo "[信息] pose_bridge 已退出，等待 FAST-LIO mapping 进程完成并保存地图..."
      echo "[信息] 提示：如果 rosbag 已停止，mapping 进程会自动检测并保存地图"
      # Wait for mapping process to finish (it will detect ros::ok() == false and save map)
      # Give it up to 30 seconds to detect rosbag stop and save map
      local wait_count=0
      local max_wait=60  # 30秒
      while kill -0 "$MAPPING_PID" 2>/dev/null && [ $wait_count -lt $max_wait ]; do
        sleep 0.5
        wait_count=$((wait_count + 1))
        if [ $((wait_count % 4)) -eq 0 ]; then
          echo "[信息] 等待 mapping 进程检测 rosbag 停止并保存地图... ($((wait_count / 2))秒)"
        fi
      done
      
      # If still running after waiting, it means it didn't detect rosbag stop
      # In this case, send SIGTERM to trigger map save
      if kill -0 "$MAPPING_PID" 2>/dev/null; then
        echo "[信息] mapping 进程仍在运行，发送退出信号以保存地图..."
        kill -TERM "$MAPPING_PID" 2>/dev/null || true
        wait "$MAPPING_PID" || true
      else
        wait "$MAPPING_PID" || true
      fi
      echo "[信息] FAST-LIO mapping 进程已退出"
    else
      # Mapping already exited, verify map was saved
      echo "[信息] FAST-LIO mapping 进程已退出，验证地图保存..."
    fi
    
    # Call cleanup to verify map was saved (cleanup will handle the verification)
    cleanup
    ;;
  *)
    echo "[错误] 未知模式: $MODE" >&2
    usage >&2
    exit 1
    ;;
esac
