#!/bin/bash
# Livox Mid-360 LiDAR Diagnostic Script
# Run this on the robot to check LiDAR health

echo "=== Livox Mid-360 LiDAR Diagnostic ==="
echo ""

echo "1. Checking LiDAR ROS2 node..."
if ros2 node list 2>/dev/null | grep -q livox_mid360; then
    echo "   ✓ livox_mid360_node is running"
else
    echo "   ✗ livox_mid360_node is NOT running"
fi

echo ""
echo "2. Checking /livox/lidar topic..."
if ros2 topic list 2>/dev/null | grep -q "/livox/lidar"; then
    echo "   ✓ Topic exists"
    echo "   Measuring rate..."
    timeout 5 ros2 topic hz /livox/lidar --window 3 2>&1 | tail -1
else
    echo "   ✗ Topic /livox/lidar does NOT exist"
fi

echo ""
echo "3. Checking /livox/imu topic (built-in IMU)..."
if ros2 topic list 2>/dev/null | grep -q "/livox/imu"; then
    echo "   ✓ Topic exists"
else
    echo "   ✗ Topic /livox/imu does NOT exist (Mid-360 has built-in ICM40609 IMU)"
fi

echo ""
echo "4. Checking all active topics..."
ros2 topic list 2>/dev/null

echo ""
echo "=== Diagnostic Complete ==="
