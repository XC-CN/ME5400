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
    
    # 启动 pose_bridge 到后台，确保脚本能响应信号
    rosrun ME5400 fastlio_pose_bridge.py &
    BRIDGE_PID=$!

    cleanup() {
      echo "[信息] 正在停止 FAST-LIO 子进程..."
      
      # 停止 pose_bridge
      if [ ! -z "$BRIDGE_PID" ]; then
        kill "$BRIDGE_PID" 2>/dev/null || true
      fi

      if kill -0 "$MAPPING_PID" 2>/dev/null; then
        echo "[信息] 正在停止 FAST-LIO mapping 节点 (PID=$MAPPING_PID)，等待地图保存..."
        
        # 方案A: 尝试使用 rosnode kill 优雅停止节点 (规避 roslaunch 强制超时)
        # 如果 rosnode 失败 (例如 master 已死), 则回退到 SIGTERM 给脚本启动的 roslaunch 进程
        if ! rosnode kill /laserMapping 2>/dev/null; then
            echo "[警告] rosnode kill 失败，尝试发送 SIGTERM 给 roslaunch 进程..."
            kill -TERM "$MAPPING_PID" 2>/dev/null || true
        else
            echo "[信息] 已发送停止命令给 /laserMapping 节点"
        fi
        
        # 等待进程退出，最多 45 秒 (给地图保存留足时间)
        local wait_count=0
        local max_wait=90  # 45秒 = 90 * 0.5秒
        
        while kill -0 "$MAPPING_PID" 2>/dev/null && [ $wait_count -lt $max_wait ]; do
          sleep 0.5
          wait_count=$((wait_count + 1))
          if [ $((wait_count % 10)) -eq 0 ]; then
             echo "[信息] 等待 FastLIO 退出中... ($((wait_count / 2))s / 45s)"
          fi
        done

        # 如果进程仍在运行，强制终止 (这是最后手段)
        if kill -0 "$MAPPING_PID" 2>/dev/null; then
          echo "[警告] 进程未在 45 秒内退出，强制终止"
          kill -KILL "$MAPPING_PID" 2>/dev/null || true
        else
           echo "[信息] FAST-LIO mapping 进程已退出"
        fi
        
        # 验证地图文件是否保存成功
        verify_map_saved
      else
        # 即使进程已经退出，也验证一下
        verify_map_saved
      fi
    }
    
    verify_map_saved() {
      # 查找 PCD 目录下最新修改的 .pcd 文件
      local pcd_dir="$ROOT_DIR/PCD"
      mkdir -p "$pcd_dir"
      
      # 使用 ls -t 找到最新的 pcd 文件
      # 注意：ls output 并不总是可靠，但在简单脚本中通常可用。
      # find 方案更健壮但语法在不同系统不仅相同。这里假设 Linux 环境。
      local newest_pcd=$(ls -t "$pcd_dir"/*.pcd 2>/dev/null | head -n 1)
      
      if [ -z "$newest_pcd" ]; then
        echo "[警告] ⚠ PCD 目录为空或未找到 .pcd 文件: $pcd_dir"
        echo "[警告] ⚠ 可能原因：1) 点云数据为空 2) 保存被中断 3) 配置中pcd_save_en=false"
        return 1
      fi
      
      local pcd_file="$newest_pcd"
      
      if [ -f "$pcd_file" ]; then
        local file_size=$(stat -f%z "$pcd_file" 2>/dev/null || stat -c%s "$pcd_file" 2>/dev/null || echo "0")
        local file_size_mb=$((file_size / 1024 / 1024))
        
        if [ "$file_size" -gt 0 ]; then
          # Check modification time to ensure it's a new file (verify recently modified)
          local current_time=$(date +%s)
          local file_mtime=$(stat -c %Y "$pcd_file" 2>/dev/null || stat -f %m "$pcd_file" 2>/dev/null || echo "0")
          local time_diff=$((current_time - file_mtime))
          
          # Allow up to 120 seconds delay (relaxed check)
          if [ "$time_diff" -gt 120 ]; then
            echo "[错误] ✗ 找到的最新地图文件似乎是旧文件！"
            echo "        文件: $pcd_file"
            echo "        文件修改时间: $(date -d @$file_mtime '+%Y-%m-%d %H:%M:%S')"
            echo "        当前系统时间: $(date -d @$current_time '+%Y-%m-%d %H:%M:%S')"
            echo "        时间差异: ${time_diff} 秒 (阈值: 120秒)"
            echo "        建议：检查 FAST-LIO 是否成功保存了地图，或尝试手动删除 PCD 目录下旧文件后重试"
            # return 1 # 暂时不报错退出，只警告，方便后续重命名逻辑尝试
          else
            echo "[成功] ✓ 地图文件已保存: $pcd_file"
            echo "[成功] ✓ 文件大小: ${file_size_mb} MB"
            
            # 为了兼容 run_all.sh 的重命名逻辑，我们可以提示或软链接
            # 但 run_all.sh 里的逻辑是找 scans.pcd。为了兼容，我们把这个最新文件复制/重命名为 scans.pcd (如果它不是叫 scans.pcd)
            # 或者，run_all.sh 也应该改为查找最新文件? 
            # 暂时保持 run_all.sh 的逻辑简单，这里做一个软链接或者拷贝?
            # 最好是 run_all.sh 也修改。但在此处，我们至少把 scans.pcd 指向最新的
            
            if [[ "$(basename "$pcd_file")" != "scans.pcd" ]]; then
                echo "[信息] 将最新文件更新为 scans.pcd 以供后续脚本使用..."
                mv "$pcd_file" "$pcd_dir/scans.pcd"
            fi
          fi
        else
          echo "[错误] ✗ 地图文件存在但大小为0，保存可能失败: $pcd_file"
          return 1
        fi
      else
        echo "[警告] ⚠ 文件检查异常"
      fi
    }
    trap cleanup EXIT INT TERM
    
    # 等待后台进程
    # 只要其中一个退出，或者收到信号，wait 就会返回
    wait $MAPPING_PID $BRIDGE_PID
    
    # 正常退出前的清理
    cleanup
    ;;
  *)
    echo "[错误] 未知模式: $MODE" >&2
    usage >&2
    exit 1
    ;;
esac
