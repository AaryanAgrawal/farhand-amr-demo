#!/bin/bash
# Simulate fault modes on BCR-001 via ROS2 parameter injection
# Usage: simulate_fault.sh <fault_type> [<fault_type2> ...]
# Types: drive, camera, imu, stereo, gripper, battery, lidar_occlusion, lidar_intermittent
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash

if [ $# -eq 0 ]; then
    echo "Usage: $0 <fault_type> [<fault_type2> ...]"
    echo "Types: drive, camera, imu, stereo, gripper, battery, lidar_occlusion, lidar_intermittent"
    exit 1
fi

for fault in "$@"; do
    case $fault in
        drive)
            ros2 param set /bcr_bot/drive_node fault.motor_overcurrent true 2>/dev/null
            echo "[FAULT] motor_overcurrent active — motor controller fault 0x03, emergency stop"
            ;;
        camera)
            ros2 param set /bcr_bot/depth_camera_node fault.usb_disconnect true 2>/dev/null
            echo "[FAULT] usb_disconnect active — zero frame data, USB disconnect"
            ;;
        imu)
            ros2 param set /bcr_bot/imu_node fault.degraded true 2>/dev/null
            echo "[FAULT] IMU degraded active — 1Hz rate, high noise, yaw drift"
            ;;
        stereo)
            ros2 param set /bcr_bot/stereo_camera_node fault.desync true 2>/dev/null
            echo "[FAULT] stereo desync active — right camera 3Hz (left 10Hz)"
            ;;
        gripper)
            ros2 param set /bcr_bot/payload_node fault.gripper_stuck true 2>/dev/null
            echo "[FAULT] gripper_stuck active — gripper stuck, weight sensor disconnected"
            ;;
        battery)
            ros2 param set /bcr_bot/battery_node fault.critical true 2>/dev/null
            echo "[FAULT] battery critical active — 3%, 10.2V, charger fault"
            ;;
        lidar_occlusion)
            ros2 param set /bcr_bot/lidar_2d_node fault.partial_occlusion true 2>/dev/null
            echo "[FAULT] partial occlusion active — 45 degree dead zone"
            ;;
        lidar_intermittent)
            ros2 param set /bcr_bot/lidar_2d_node fault.intermittent true 2>/dev/null
            echo "[FAULT] intermittent dropout active — 5s dropout every 30s"
            ;;
        *)
            echo "Unknown fault: $fault"
            echo "Valid: drive, camera, imu, stereo, gripper, battery, lidar_occlusion, lidar_intermittent"
            ;;
    esac
done
echo ""
echo "Active nodes:"
ros2 node list 2>/dev/null
