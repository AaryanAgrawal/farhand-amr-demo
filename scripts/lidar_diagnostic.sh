#!/bin/bash
# 2D LiDAR Diagnostic Script
# Run this on the robot to check LiDAR health
source /opt/ros/humble/setup.bash 2>/dev/null
source /home/ubuntu/ros2_ws/install/setup.bash 2>/dev/null

echo "=== 2D LiDAR Diagnostic ==="
echo ""

echo "1. Checking LiDAR ROS2 node..."
if ros2 node list 2>/dev/null | grep -q two_d_lidar; then
    echo "   [OK] two_d_lidar_node is running"
else
    echo "   [FAIL] two_d_lidar_node is NOT running"
fi

echo ""
echo "2. Checking /bcr_bot/scan topic..."
if ros2 topic list 2>/dev/null | grep -q "/bcr_bot/scan"; then
    echo "   [OK] Topic exists"
    echo "   Measuring rate..."
    timeout 5 ros2 topic hz /bcr_bot/scan --window 3 2>&1 | tail -1
else
    echo "   [FAIL] Topic /bcr_bot/scan does NOT exist"
fi

echo ""
echo "3. Checking all active topics..."
ros2 topic list 2>/dev/null

echo ""
echo "=== Diagnostic Complete ==="
