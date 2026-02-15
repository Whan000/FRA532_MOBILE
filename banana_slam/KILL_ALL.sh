#!/bin/bash
# Nuclear option - kill all ROS2 and related processes
# Enhanced version with graceful shutdown, cleanup, and verification

set +e  # Don't exit on errors

echo "=========================================="
echo "¥ KILLING ALL ROS2/ODOMETRY PROCESSES"
echo "=========================================="
echo ""

# Function to kill processes gracefully, then forcefully
kill_process() {
    local pattern="$1"
    local description="$2"

    # Find matching processes
    local pids=$(pgrep -f "$pattern" 2>/dev/null)

    if [ -n "$pids" ]; then
        echo "[$description] Found processes: $pids"

        # Try graceful shutdown first (SIGTERM)
        echo "  †’ Sending SIGTERM (graceful)..."
        pkill -15 -f "$pattern" 2>/dev/null
        sleep 0.5

        # Check if still alive
        pids=$(pgrep -f "$pattern" 2>/dev/null)
        if [ -n "$pids" ]; then
            echo "  †’ Still alive, sending SIGKILL (force)..."
            pkill -9 -f "$pattern" 2>/dev/null
            sleep 0.2
        else
            echo "   Gracefully terminated"
        fi
    fi
}

# Function to kill by exact process name
kill_by_name() {
    local name="$1"
    local description="$2"

    local pids=$(pgrep "^${name}$" 2>/dev/null)

    if [ -n "$pids" ]; then
        echo "[$description] Found: $pids"
        kill -15 $pids 2>/dev/null
        sleep 0.3

        # Force kill if still alive
        pids=$(pgrep "^${name}$" 2>/dev/null)
        if [ -n "$pids" ]; then
            kill -9 $pids 2>/dev/null
        fi
    fi
}

echo "=== Stage 1: ROS2 Core Processes ==="
kill_process "ros2 daemon" "ROS2 Daemon"
kill_process "_ros2_daemon" "ROS2 Daemon (alternative)"
kill_by_name "rviz2" "RViz2"
kill_by_name "rqt" "RQt"

echo ""
echo "=== Stage 2: Bag Files ==="
kill_process "ros2 bag play" "Bag Player"
kill_process "ros2 bag record" "Bag Recorder"
kill_process "ros2 bag" "Any Bag Command"

echo ""
echo "=== Stage 3: Launch & Run ==="
kill_process "ros2 launch" "ROS2 Launch"
kill_process "ros2 run" "ROS2 Run"

echo ""
echo "=== Stage 4: Odometry Nodes ==="
kill_process "ekf_node" "EKF Node"
kill_process "icp_node" "ICP Node"
kill_process "simple_icp_node" "Simple ICP Node"
kill_process "lidar_filter" "LiDAR Filter"
kill_process "trajectory_plotter" "Trajectory Plotter"
kill_process "analyze_live_odom" "Live Odometry Analyzer"
kill_process "data_logger" "Data Logger"

echo ""
echo "=== Stage 5: SLAM & Mapping ==="
kill_process "slam_toolbox" "SLAM Toolbox"
kill_process "async_slam_toolbox" "Async SLAM Toolbox"
kill_process "sync_slam_toolbox" "Sync SLAM Toolbox"
kill_process "map_server" "Map Server"

echo ""
echo "=== Stage 6: Optimization Scripts ==="
kill_process "bayesian_auto_tune" "Bayesian Tuner"
kill_process "auto_tune" "Auto Tuner"
kill_process "grid_search" "Grid Search"

echo ""
echo "=== Stage 7: Package-Specific Nodes ==="
kill_process "banana_odom" "Banana Odom Package"
kill_process "icp_odometry" "ICP Odometry Package"

echo ""
echo "=== Stage 8: Matplotlib/GUI Processes ==="
# Trajectory plotter uses matplotlib which can leave processes
pkill -9 -f "matplotlib" 2>/dev/null
# Kill any python processes running from our workspace
kill_process "/home/zenter/Desktop/mobile/banana_slam" "Workspace Python Processes"

echo ""
echo "=== Stage 9: Catch-All ROS2 & Python ==="
# Nuclear option - kill ALL ros2 commands
killall -9 ros2 2>/dev/null
# Kill all python3 processes that mention our packages
pkill -9 -f "banana_slam" 2>/dev/null

echo ""
echo "=== Stage 10: Cleanup Runtime Artifacts ==="
echo "Cleaning ROS2 shared memory..."
rm -f /dev/shm/fastrtps_* 2>/dev/null
rm -f /dev/shm/sem.ros2_* 2>/dev/null
rm -rf /dev/shm/Fast-DDS* 2>/dev/null

echo "Cleaning ROS2 temp files..."
rm -rf /tmp/.ros* 2>/dev/null
rm -rf /tmp/launch_* 2>/dev/null

echo "Cleaning DDS discovery files..."
rm -rf /tmp/fast-rtps-* 2>/dev/null
rm -rf /tmp/fastrtps-* 2>/dev/null

echo "Cleaning matplotlib cache..."
rm -rf /tmp/matplotlib-* 2>/dev/null

echo ""
echo "=== Verification ==="
echo "Checking for remaining ROS2/odometry processes..."
remaining=$(ps aux | grep -E "ros2|ekf_node|icp_node|slam_toolbox|rviz2|banana_odom|trajectory_plotter" | grep -v grep | grep -v "KILL_ALL")

if [ -n "$remaining" ]; then
    echo "  WARNING: Some processes still running:"
    echo "$remaining"
    echo ""
    echo "Attempting final nuclear cleanup..."

    # Extract PIDs and kill
    echo "$remaining" | awk '{print $2}' | xargs -r kill -9 2>/dev/null
    sleep 0.5

    # Check again
    remaining=$(ps aux | grep -E "ros2|ekf_node|icp_node|slam_toolbox|rviz2|banana_odom|trajectory_plotter" | grep -v grep | grep -v "KILL_ALL")
    if [ -n "$remaining" ]; then
        echo " FAILED: These processes won't die:"
        echo "$remaining"
    else
        echo " All processes killed after nuclear cleanup"
    fi
else
    echo " No remaining processes found"
fi

# Check for zombie processes
zombies=$(ps aux | awk '{if ($8=="Z") print $0}')
if [ -n "$zombies" ]; then
    echo ""
    echo "  Found zombie processes (will clean up on their own):"
    echo "$zombies"
fi

echo ""
echo "=== Resource Cleanup Summary ==="
echo " Shared memory cleaned"
echo " Temp files removed"
echo " DDS artifacts purged"

echo ""
echo "=========================================="
echo " KILL_ALL COMPLETE!"
echo "=========================================="
echo ""
echo "System is clean. You can now run:"
echo "  bash run_complete_test.sh"
echo "  OR"
echo "  bash run_odom_test.sh"
echo ""
