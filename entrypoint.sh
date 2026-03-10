#!/bin/bash
set -e

# Source ROS2 and workspace overlay
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash

# Start SSH server
/usr/sbin/sshd

# Ensure logs directory exists and is writable
mkdir -p /home/ubuntu/logs
chown -R ubuntu:ubuntu /home/ubuntu/logs

echo "============================================"
echo "  BCR-001 AMR Simulator"
echo "  ROS2 Humble | Cyclone DDS | colcon"
echo "  Firmware v2.4.1"
echo "============================================"
echo ""
echo "Starting base nodes (LiDAR OFF)..."

# Launch all nodes except LiDAR via ros2 launch
ros2 launch bcr_bot bringup.launch.py &

sleep 3

echo "Nodes: robot_state_publisher, depth_camera, stereo_camera, drive, imu, battery, nav_sim, payload"
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
