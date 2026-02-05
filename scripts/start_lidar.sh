#!/bin/bash
# Start the Livox Mid-360 LiDAR node
# Run this after replacing the LiDAR hardware

echo "Starting Livox Mid-360 LiDAR node..."
source /opt/ros/humble/setup.bash
python3 /home/ubuntu/nodes/lidar_node.py &
LIDAR_PID=$!
echo "LiDAR node started (PID: $LIDAR_PID)"
echo "Waiting 3 seconds for node to initialize..."
sleep 3

if ros2 topic list 2>/dev/null | grep -q "/livox/lidar"; then
    echo "✓ /livox/lidar topic is now active"
else
    echo "✗ Failed to start LiDAR node"
fi
