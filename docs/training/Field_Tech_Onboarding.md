# Field Technician Onboarding Guide

**Robot:** BCR-001 (Differential Drive AMR)
**Last Updated:** 2026-03-06

## 1. Connecting to BCR-001

The robot runs ROS2 Humble inside a Docker container. Access is via SSH:

```bash
ssh ubuntu@<robot_ip> -p 2222
# Password: ubuntu
```

The robot IP is configured in the Farhand platform under Sites > Robot Configuration.

Once connected, ROS2 is automatically sourced. You can immediately run commands like:

```bash
ros2 node list        # List running nodes
ros2 topic list       # List active topics
ros2 topic echo <topic> --once   # Read one message from a topic
ros2 topic hz <topic> --window 3 # Measure publishing rate
```

## 2. System Architecture

BCR-001 runs 7 core nodes (LiDAR optional, 8th node):

| Node | Topic(s) | Rate | Purpose |
|------|----------|------|---------|
| depth_camera_node | /bcr_bot/camera/* | 30Hz | Forward-facing depth camera |
| stereo_camera_node | /bcr_bot/stereo/left/*, right/* | 10Hz | Stereo pair for 3D mapping |
| drive_node | /bcr_bot/odom, /bcr_bot/cmd_vel, /bcr_bot/joint_states | 50Hz | Differential drive motors |
| imu_node | /bcr_bot/imu | 5Hz | Inertial measurement unit |
| battery_node | /bcr_bot/battery_state | 1Hz | Battery monitoring |
| nav_sim_node | /bcr_bot/nav_status | 1Hz | Navigation state machine |
| payload_node | /bcr_bot/payload/status, gripper_state | 1-2Hz | Pallet gripper system |
| lidar_2d_node | /bcr_bot/scan | 30Hz | 2D LiDAR (OFF by default) |

## 3. Diagnostic Scripts

All scripts are in `~/scripts/`:

| Script | Purpose | When to Use |
|--------|---------|-------------|
| `sensor_health_check.sh` | Check all sensors at once | Start of shift, after any fault |
| `drive_diagnostic.sh` | Drive motor deep check | Drive fault codes, encoder issues |
| `battery_report.sh` | Battery state and history | Low battery alerts, charging issues |
| `camera_diagnostic.sh` | Camera rates and sync | Image quality issues, stereo desync |
| `lidar_diagnostic.sh` | LiDAR status check | LiDAR offline or degraded |
| `start_lidar.sh` | Start LiDAR node | When LiDAR is needed for navigation |
| `simulate_fault.sh` | Trigger test faults | Testing/training only |
| `clear_faults.sh` | Clear all faults, restart clean | After fault testing |

## 4. Reading Logs

All logs are in `~/logs/`:

```bash
# Recent entries from any log
tail -20 ~/logs/drive.log
tail -20 ~/logs/battery.log
tail -20 ~/logs/sensor_health.log
tail -20 ~/logs/navigation.log
tail -20 ~/logs/payload.log
tail -20 ~/logs/system_startup.log

# Search for errors
grep "ERROR\|CRITICAL" ~/logs/drive.log
grep "WARNING" ~/logs/sensor_health.log
```

## 5. Common ROS2 Commands

```bash
# Check if a specific node is running
ros2 node list | grep drive

# Get detailed info about a topic
ros2 topic info /bcr_bot/odom

# Watch a topic in real-time (Ctrl+C to stop)
ros2 topic echo /bcr_bot/battery_state

# Measure topic publishing rate
timeout 6 ros2 topic hz /bcr_bot/imu --window 3

# Send a velocity command (careful!)
ros2 topic pub /bcr_bot/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.05}, angular: {z: 0.0}}" --once
```

## 6. When to Escalate

**Escalate immediately** (do not attempt to fix):
- Motor fault code 0x05 (Hall sensor failure)
- Burning smell from any component
- Physical damage to robot body, wheels, or sensors
- Battery voltage below 10V
- Hydraulic/pneumatic fluid leak
- Robot involved in collision with person or equipment

**Try to resolve first, escalate if unsuccessful**:
- Sensor offline (restart node first)
- Gripper fault (check cables, restart node)
- Navigation stuck (check LiDAR, restart nav node)
- Battery not charging (check dock alignment, power cable)

## 7. Safety Reminders

- Always verify E-stop before physical inspection
- Never reach into the gripper mechanism while powered
- Maximum payload: 30kg — never exceed
- Minimum clearance zone: 1m during operation, 3m during fault investigation
- Report ALL incidents, even minor ones
