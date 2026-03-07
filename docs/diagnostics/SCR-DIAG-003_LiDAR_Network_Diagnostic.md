# Script: 2D LiDAR Diagnostic

**Script ID:** SCR-DIAG-003
**Category:** Diagnostics
**Version:** 2.0
**Estimated Duration:** 30 seconds
**Requires Credentials:** Yes (SSH to robot)

## Description
Diagnostic script for the BCR Bot 2D LiDAR sensor. Checks ROS2 node status, scan topic health, and scan rate.

## Parameters
| Name | Label | Type | Required | Default | Description |
|------|-------|------|----------|---------|-------------|
| robot_ip | Robot IP | string | yes | 192.168.1.50 | Robot's IP address |
| robot_user | Robot User | string | yes | ubuntu | SSH username |

## Steps

### Step 1: Check 2D LiDAR ROS2 Node
- **Type:** ssh
- **Command:** `ros2 node list 2>/dev/null | grep -i two_d_lidar`
- **Timeout:** 10s
- **Expected Exit Code:** 0
- **On Failure:** continue
- **Capture Output:** node_status

### Step 2: Check Scan Topic Rate
- **Type:** ssh
- **Command:** `timeout 10 ros2 topic hz /bcr_bot/scan --window 5 2>&1 | tail -3`
- **Timeout:** 15s
- **On Failure:** continue
- **Capture Output:** scan_rate

### Step 3: Check Scan Topic Type
- **Type:** ssh
- **Command:** `ros2 topic info /bcr_bot/scan 2>/dev/null`
- **Timeout:** 10s
- **On Failure:** continue
- **Capture Output:** scan_info

### Step 4: Check All Active Topics
- **Type:** ssh
- **Command:** `ros2 topic list 2>/dev/null`
- **Timeout:** 10s
- **On Failure:** continue
- **Capture Output:** all_topics

## Outputs
| Name | Label | Type | Source |
|------|-------|------|--------|
| node_status | ROS2 Node Running | pass_fail | Step 1 exit code |
| scan_rate | Scan Hz | number | Step 2 output |
| scan_info | Topic Info | text | Step 3 output |
