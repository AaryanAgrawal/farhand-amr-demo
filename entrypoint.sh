#!/bin/bash
set -e

# Source ROS2
source /opt/ros/humble/setup.bash

# Start SSH server
/usr/sbin/sshd

echo "============================================"
echo "  Farhand BCR-001 AMR Simulator"
echo "  ROS2 Humble | Cyclone DDS"
echo "============================================"
echo ""
echo "Starting base nodes (LiDAR OFF)..."

# Start all nodes EXCEPT LiDAR
python3 /home/ubuntu/nodes/camera_node.py &
python3 /home/ubuntu/nodes/drive_node.py &
python3 /home/ubuntu/nodes/imu_node.py &
python3 /home/ubuntu/nodes/battery_node.py &

echo "Nodes started: camera, drive, imu, battery"
echo "LiDAR is NOT running (simulate replacement scenario)"
echo ""
echo "SSH access: ssh ubuntu@localhost -p 2222 (password: ubuntu)"
echo ""

# Keep container alive
wait
