# Script: Livox Mid-360 Network Diagnostic

**Script ID:** SCR-DIAG-003
**Category:** Diagnostics
**Version:** 1.0
**Estimated Duration:** 30 seconds
**Requires Credentials:** Yes (SSH to robot)

## Description
Comprehensive diagnostic script for the Livox Mid-360 LiDAR sensor. Checks network connectivity, ROS2 driver status, point cloud health, and IMU data stream.

## Parameters
| Name | Label | Type | Required | Default | Description |
|------|-------|------|----------|---------|-------------|
| robot_ip | Robot IP | string | yes | 192.168.1.50 | Robot's IP address |
| robot_user | Robot User | string | yes | ubuntu | SSH username |
| lidar_ip | LiDAR IP | string | yes | 192.168.1.181 | LiDAR sensor IP |

## Steps

### Step 1: Ping LiDAR Sensor
- **Type:** ssh
- **Command:** `ping -c 3 {{lidar_ip}}`
- **Timeout:** 10s
- **Expected Exit Code:** 0
- **On Failure:** continue
- **Capture Output:** ping_result

### Step 2: Check Livox ROS2 Node
- **Type:** ssh
- **Command:** `ros2 node list 2>/dev/null | grep -i livox`
- **Timeout:** 10s
- **Expected Exit Code:** 0
- **On Failure:** continue
- **Capture Output:** node_status

### Step 3: Check Point Cloud Topic Rate
- **Type:** ssh
- **Command:** `timeout 10 ros2 topic hz /livox/lidar --window 5 2>&1 | tail -3`
- **Timeout:** 15s
- **On Failure:** continue
- **Capture Output:** pointcloud_rate

### Step 4: Check IMU Topic Rate
- **Type:** ssh
- **Command:** `timeout 10 ros2 topic hz /livox/imu --window 5 2>&1 | tail -3`
- **Timeout:** 15s
- **On Failure:** continue
- **Capture Output:** imu_rate

### Step 5: Read LiDAR Config
- **Type:** ssh
- **Command:** `cat /opt/livox/config/MID360_config.json 2>/dev/null || echo "Config file not found"`
- **Timeout:** 5s
- **On Failure:** continue
- **Capture Output:** lidar_config

### Step 6: Check Livox Driver Service
- **Type:** ssh
- **Command:** `systemctl is-active livox_ros2_driver 2>/dev/null || echo "Service not found"`
- **Timeout:** 5s
- **On Failure:** continue
- **Capture Output:** driver_status

## Outputs
| Name | Label | Type | Source |
|------|-------|------|--------|
| ping_result | LiDAR Reachable | pass_fail | Step 1 exit code |
| node_status | ROS2 Node Running | pass_fail | Step 2 exit code |
| pointcloud_rate | Point Cloud Hz | number | Step 3 output |
| imu_rate | IMU Hz | number | Step 4 output |
| driver_status | Driver Service | text | Step 6 output |
