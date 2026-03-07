#!/bin/bash
# Start the 2D LiDAR node
# Run this after replacing the LiDAR hardware

echo "Starting 2D LiDAR node..."
source /opt/ros/humble/setup.bash
python3 /home/ubuntu/nodes/lidar_2d_node.py &
LIDAR_PID=$!
echo "LiDAR node started (PID: $LIDAR_PID)"
echo "Waiting 3 seconds for node to initialize..."
sleep 3

if ros2 topic list 2>/dev/null | grep -q "/bcr_bot/scan"; then
    echo "[OK] /bcr_bot/scan topic is now active"
else
    echo "[FAIL] Failed to start LiDAR node"
fi
