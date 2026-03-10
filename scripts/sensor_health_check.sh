#!/bin/bash
# BCR-001 Sensor Health Check — checks all sensors and outputs formatted table
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash 2>/dev/null

echo ""
echo "  BCR-001 SENSOR HEALTH REPORT"
echo "  ============================="
echo ""
printf "  %-18s %-8s %-10s %-10s\n" "Sensor" "Status" "Rate" "Expected"
echo "  --------------------------------------------------"

check_topic() {
    local topic=$1 expected=$2 label=$3
    rate=$(timeout 6 ros2 topic hz "$topic" --window 3 2>/dev/null | grep "average" | head -1 | awk '{printf "%.1f", $3}')
    if [ -z "$rate" ] || [ "$rate" = "0.0" ]; then
        printf "  %-18s %-8s %-10s %-10s\n" "$label" "OFF" "---" "${expected}Hz"
    else
        printf "  %-18s %-8s %-10s %-10s\n" "$label" "OK" "${rate}Hz" "${expected}Hz"
    fi
}

check_topic "/bcr_bot/camera/image_raw" 30 "Depth Camera"
check_topic "/bcr_bot/stereo/left/image_raw" 10 "Stereo Left"
check_topic "/bcr_bot/stereo/right/image_raw" 10 "Stereo Right"
check_topic "/bcr_bot/imu" 5 "IMU"
check_topic "/bcr_bot/battery_state" 1 "Battery"
check_topic "/bcr_bot/scan" 30 "2D LiDAR"
check_topic "/bcr_bot/odom" 50 "Drive/Odom"
check_topic "/bcr_bot/nav_status" 1 "Navigation"
check_topic "/bcr_bot/payload/status" 1 "Payload"
check_topic "/bcr_bot/payload/gripper_state" 2 "Gripper"

echo "  --------------------------------------------------"
echo ""

total=$(ros2 topic list 2>/dev/null | wc -l)
nodes=$(ros2 node list 2>/dev/null | wc -l)
echo "  Active topics: $total | Active nodes: $nodes"
echo ""
