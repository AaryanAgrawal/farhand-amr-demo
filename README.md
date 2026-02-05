# Farhand AMR Demo (BCR-001)

Simulated ROS2 Humble AMR robot for testing the Farhand Field platform. Runs in Docker with SSH access for the Farhand Field CLI agent.

## What It Simulates

| Sensor | Topic | Rate | Node |
|--------|-------|------|------|
| Livox Mid-360 LiDAR | `/livox/lidar` | 10Hz | `lidar_node.py` |
| USB Camera | `/camera/image_raw` | 30Hz | `camera_node.py` |
| Differential Drive | `/odom`, `/joint_states` | 50Hz | `drive_node.py` |
| IMU (BNO055) | `/imu/data` | 100Hz | `imu_node.py` |
| Battery (3S LiPo) | `/battery_state` | 1Hz | `battery_node.py` |

**The LiDAR starts OFF by default** to simulate a replacement scenario.

## Quick Start

```bash
docker compose up --build
```

SSH into the robot:
```bash
ssh ubuntu@localhost -p 2222
# Password: ubuntu
```

## Demo Scenario: LiDAR Replacement

```bash
# 1. Check LiDAR status (should be DOWN)
bash ~/scripts/lidar_diagnostic.sh

# 2. Start LiDAR (simulates post-replacement)
bash ~/scripts/start_lidar.sh

# 3. Verify LiDAR is running
bash ~/scripts/lidar_diagnostic.sh
```

## Team Scripts

| Script | Purpose |
|--------|---------|
| `scripts/lidar_diagnostic.sh` | Check LiDAR node, topic, and rate |
| `scripts/start_lidar.sh` | Start the LiDAR node after replacement |

## Network

- SSH: `localhost:2222` (user: `ubuntu`, password: `ubuntu`)
- ROS2 DDS: Cyclone DDS, domain 0

## For Farhand Field CLI

```bash
farhand-field --robot-ip=localhost --robot-user=ubuntu
```

The CLI agent can SSH in and run diagnostics, start the LiDAR, verify the fix — all through natural language.
