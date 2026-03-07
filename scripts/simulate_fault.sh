#!/bin/bash
# Simulate fault modes on BCR-001
# Usage: simulate_fault.sh <fault_type> [<fault_type2> ...]
# Types: drive, camera, imu, stereo, gripper, battery, lidar_occlusion, lidar_intermittent
source /opt/ros/humble/setup.bash

if [ $# -eq 0 ]; then
    echo "Usage: $0 <fault_type> [<fault_type2> ...]"
    echo "Types: drive, camera, imu, stereo, gripper, battery, lidar_occlusion, lidar_intermittent"
    exit 1
fi

cd /home/ubuntu

for fault in "$@"; do
    case $fault in
        drive)
            pkill -f drive_node.py 2>/dev/null
            sleep 1
            DRIVE_FAULT=1 python3 nodes/drive_node.py &
            echo "[FAULT] DRIVE_FAULT active — motor controller fault 0x03, emergency stop"
            ;;
        camera)
            pkill -f depth_camera_node.py 2>/dev/null
            sleep 1
            CAMERA_FAULT=1 python3 nodes/depth_camera_node.py &
            echo "[FAULT] CAMERA_FAULT active — zero frame data, USB disconnect"
            ;;
        imu)
            pkill -f imu_node.py 2>/dev/null
            sleep 1
            IMU_DEGRADED=1 python3 nodes/imu_node.py &
            echo "[FAULT] IMU_DEGRADED active — 1Hz rate, high noise, yaw drift"
            ;;
        stereo)
            pkill -f stereo_camera_node.py 2>/dev/null
            sleep 1
            STEREO_DESYNC=1 python3 nodes/stereo_camera_node.py &
            echo "[FAULT] STEREO_DESYNC active — right camera 3Hz (left 10Hz)"
            ;;
        gripper)
            pkill -f payload_node.py 2>/dev/null
            sleep 1
            GRIPPER_FAULT=1 python3 nodes/payload_node.py &
            echo "[FAULT] GRIPPER_FAULT active — gripper stuck, weight sensor disconnected"
            ;;
        battery)
            pkill -f battery_node.py 2>/dev/null
            sleep 1
            BATTERY_CRITICAL=1 python3 nodes/battery_node.py &
            echo "[FAULT] BATTERY_CRITICAL active — 3%, 10.2V, charger fault"
            ;;
        lidar_occlusion)
            pkill -f lidar_2d_node.py 2>/dev/null
            sleep 1
            LIDAR_PARTIAL_OCCLUSION=1 python3 nodes/lidar_2d_node.py &
            echo "[FAULT] LIDAR_PARTIAL_OCCLUSION active — 45 degree dead zone"
            ;;
        lidar_intermittent)
            pkill -f lidar_2d_node.py 2>/dev/null
            sleep 1
            LIDAR_INTERMITTENT=1 python3 nodes/lidar_2d_node.py &
            echo "[FAULT] LIDAR_INTERMITTENT active — 5s dropout every 30s"
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
