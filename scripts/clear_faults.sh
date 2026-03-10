#!/bin/bash
# Clear all fault modes via ROS2 parameters — no process restart needed
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash

echo "Clearing all fault modes..."

ros2 param set /bcr_bot/battery_node fault.critical false 2>/dev/null
ros2 param set /bcr_bot/depth_camera_node fault.usb_disconnect false 2>/dev/null
ros2 param set /bcr_bot/stereo_camera_node fault.desync false 2>/dev/null
ros2 param set /bcr_bot/drive_node fault.motor_overcurrent false 2>/dev/null
ros2 param set /bcr_bot/imu_node fault.degraded false 2>/dev/null
ros2 param set /bcr_bot/lidar_2d_node fault.partial_occlusion false 2>/dev/null
ros2 param set /bcr_bot/lidar_2d_node fault.intermittent false 2>/dev/null
ros2 param set /bcr_bot/nav_sim_node fault.drive_fault false 2>/dev/null
ros2 param set /bcr_bot/payload_node fault.gripper_stuck false 2>/dev/null

echo ""
echo "All faults cleared. Nodes continue running in normal mode."
echo "Active nodes:"
ros2 node list 2>/dev/null
