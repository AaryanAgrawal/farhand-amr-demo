#!/bin/bash
# Clear all fault modes and restart all nodes in normal mode
source /opt/ros/humble/setup.bash

echo "Clearing all fault modes..."
pkill -f "python3 nodes/" 2>/dev/null
pkill -f "python3 /home/ubuntu/nodes/" 2>/dev/null
sleep 2

cd /home/ubuntu

# Restart all base nodes (no faults, LiDAR stays off by default)
python3 nodes/depth_camera_node.py &
python3 nodes/stereo_camera_node.py &
python3 nodes/drive_node.py &
python3 nodes/imu_node.py &
python3 nodes/battery_node.py &
python3 nodes/nav_sim_node.py &
python3 nodes/payload_node.py &

sleep 3
echo ""
echo "All nodes restarted in normal mode (no faults)."
echo "LiDAR remains OFF (start manually with ~/scripts/start_lidar.sh)"
echo ""
echo "Active nodes:"
ros2 node list 2>/dev/null
