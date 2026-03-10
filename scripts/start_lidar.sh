#!/bin/bash
# Start the 2D LiDAR node via ros2 launch
# Run this after replacing the LiDAR hardware

echo "Starting 2D LiDAR node..."
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash

ros2 launch bcr_bot lidar.launch.py &
echo "LiDAR launch started"
echo "Waiting 3 seconds for node to initialize..."
sleep 3

if ros2 topic list 2>/dev/null | grep -q "/bcr_bot/scan"; then
    echo "[OK] /bcr_bot/scan topic is now active"
else
    echo "[FAIL] Failed to start LiDAR node"
fi
