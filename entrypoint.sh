#!/bin/bash
set -e

# Source ROS2
source /opt/ros/humble/setup.bash

# Start SSH server
/usr/sbin/sshd

# Ensure logs directory exists and is writable
mkdir -p /home/ubuntu/logs
chown -R ubuntu:ubuntu /home/ubuntu/logs

echo "============================================"
echo "  BCR-001 AMR Simulator"
echo "  ROS2 Humble | Cyclone DDS"
echo "  Firmware v2.4.1"
echo "============================================"
echo ""
echo "Starting base nodes (LiDAR OFF)..."

# Start all nodes EXCEPT LiDAR (7 nodes)
python3 /home/ubuntu/nodes/depth_camera_node.py &
python3 /home/ubuntu/nodes/stereo_camera_node.py &
python3 /home/ubuntu/nodes/drive_node.py &
python3 /home/ubuntu/nodes/imu_node.py &
python3 /home/ubuntu/nodes/battery_node.py &
python3 /home/ubuntu/nodes/nav_sim_node.py &
python3 /home/ubuntu/nodes/payload_node.py &

echo "Nodes started: depth_camera, stereo_camera, drive, imu, battery, nav_sim, payload"
echo "LiDAR is NOT running (start with ~/scripts/start_lidar.sh)"
echo ""
echo "Diagnostic scripts: ~/scripts/"
echo "Logs: ~/logs/"
echo "Docs: ~/docs/"
echo ""
echo "SSH access: ssh ubuntu@localhost -p 2222 (password: ubuntu)"
echo ""

# Keep container alive
wait
