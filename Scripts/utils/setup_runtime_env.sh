#!/bin/bash

# Source this file from launcher scripts to make the runtime environment
# self-contained. The order matters: activate Conda first, then restore ROS.

ME5400_SETUP_SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
ME5400_SETUP_PROJECT_ROOT=$(cd "$ME5400_SETUP_SCRIPT_DIR/../.." && pwd)
ME5400_SETUP_CATKIN_WS="$ME5400_SETUP_PROJECT_ROOT/catkin_ws"
ME5400_SETUP_CONDA_SH="${HOME}/miniconda3/etc/profile.d/conda.sh"
ME5400_SETUP_CONDA_ENV_NAME="${ME5400_CONDA_ENV:-ME5400}"

if [[ -f "$ME5400_SETUP_CONDA_SH" ]]; then
    HAD_NOUNSET=0
    if [[ $- == *u* ]]; then
        HAD_NOUNSET=1
    fi
    set +u
    source "$ME5400_SETUP_CONDA_SH"
    if [[ "${CONDA_DEFAULT_ENV:-}" != "$ME5400_SETUP_CONDA_ENV_NAME" ]]; then
        conda activate "$ME5400_SETUP_CONDA_ENV_NAME"
    fi
    if [[ "$HAD_NOUNSET" == "1" ]]; then
        set -u
    fi
else
    echo "[警告] 未找到 conda 初始化脚本: $ME5400_SETUP_CONDA_SH" >&2
fi

source /opt/ros/noetic/setup.bash

if [[ -d "$ME5400_SETUP_CATKIN_WS/devel" ]]; then
    source "$ME5400_SETUP_CATKIN_WS/devel/setup.bash"
fi
