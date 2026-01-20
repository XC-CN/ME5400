#!/bin/bash
set -e

# get project root
SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
PROJECT_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)

# Function to handle cleanup on exit
cleanup() {
    echo -e "\n[INFO] Stopping all processes..."
    # Kill all child processes in the same process group
    trap - SIGTERM && kill -- -$$
}

# Trap SIGINT (Ctrl+C) and EXIT
trap cleanup SIGINT EXIT

echo "[INFO] Starting ME5400 Full Pipeline (Excluding Step 3)..."

# 1. Start roscore
echo "[STEP 1] Starting roscore..."
roscore &
ROSCORE_PID=$!
sleep 5  # Wait for roscore to initialize

# 2. Build catkin workspace (Fast if already built)
echo "[STEP 2] Building/Verifying catkin workspace..."
"$PROJECT_ROOT/Scripts/utils/build_catkin_ws.sh"

# 4. Start PointPillars Node
echo "[STEP 4] Starting PointPillars Node..."
"$PROJECT_ROOT/Scripts/pointpillars/run_pointpillars_node.sh" &
PP_PID=$!
sleep 5 # Wait for model to load

# 5. Start FAST-LIO System
echo "[STEP 5] Starting FAST-LIO (Mapping + Pose Bridge)..."
"$PROJECT_ROOT/Scripts/fastlio/run_fastlio.sh" both &
FL_PID=$!
sleep 3

# 6. Start MCTrack Online Node
echo "[STEP 6] Starting MCTrack Online Node..."
"$PROJECT_ROOT/Scripts/mctrack/run_mctrack_online_node.sh" &
MC_PID=$!
sleep 2

# 8. Open RViz
echo "[STEP 8] Starting RViz..."
"$PROJECT_ROOT/Scripts/rviz/run_rviz.sh" &
RVIZ_PID=$!
sleep 5

# 7. Play Rosbag
# Default sequence is 0020
SEQ_ID=${1:-"0020"}
BAG_FILE="$PROJECT_ROOT/Data_Tracking/rosbags/seq_${SEQ_ID}_nodet.bag"

if [[ ! -f "$BAG_FILE" ]]; then
    echo "[WARN] Bag file not found at $BAG_FILE"
    echo "       Please check if the sequence ID '$SEQ_ID' is correct and the bag exists in Data_Tracking/rosbags/"
    echo "       Running without bag play (Manual play required)."
    wait $RVIZ_PID
else
    echo "[STEP 7] Playing Rosbag: $BAG_FILE"
    rosparam set use_sim_time true
    rosbag play "$BAG_FILE" --clock --loop &
    BAG_PID=$!
    
    echo "========================================================="
    echo "   Pipeline Running! Press Ctrl+C to stop everything.    "
    echo "========================================================="
    
    # Wait for the bag player or user interrupt
    wait $BAG_PID
fi

# Keep script running if bag play was skipped or finished (if loop is off, but here loop is on)
wait
