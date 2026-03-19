# Farhand AMR Test Bench (BCR-001)

Dockerized ROS2 robot simulator for testing the [Farhand Field](https://field.farhand.live) platform and CLI agent. Emulates a BCR-001 AMR with 2D LiDAR, depth camera, stereo camera, IMU, differential drive, and battery. The LiDAR starts OFF to simulate a replacement scenario.

## Quick Start

```bash
docker compose up --build -d
```

SSH into the robot:
```bash
ssh ubuntu@localhost -p 2222
# Password: ubuntu
```

Verify running topics:
```bash
ros2 topic list
```

You should see `/bcr_bot/camera/*`, `/bcr_bot/stereo/*`, `/bcr_bot/odom`, `/bcr_bot/joint_states`, `/bcr_bot/imu`, `/bcr_bot/battery_state`, `/bcr_bot/cmd_vel`. Note: `/bcr_bot/scan` is NOT listed — the LiDAR is intentionally off.`

## Remote Access (Tailscale)

The test bench runs on a Mac Mini accessible via Tailscale.

| Setting | Value |
|---------|-------|
| Host | `aaryans-mac-mini` |
| Tailscale IP | `100.84.147.55` |
| SSH | `ssh ubuntu@100.84.147.55 -p 2222` |
| Password | `ubuntu` |

From any machine on the tailnet:
```bash
ssh ubuntu@100.84.147.55 -p 2222
```

## Simulated Sensors

| Sensor | Topic | Rate | Node |
|--------|-------|------|------|
| 2D LiDAR | `/bcr_bot/scan` | 30Hz | `lidar_2d_node.py` |
| Depth Camera | `/bcr_bot/camera/image_raw` | 30Hz | `depth_camera_node.py` |
| Stereo Camera | `/bcr_bot/stereo/left/image_raw`, `right/image_raw` | 10Hz | `stereo_camera_node.py` |
| Differential Drive | `/bcr_bot/odom`, `/bcr_bot/joint_states` | 50Hz | `drive_node.py` |
| IMU | `/bcr_bot/imu` | 5Hz | `imu_node.py` |
| Battery | `/bcr_bot/battery_state` | 1Hz | `battery_node.py` |

**The LiDAR starts OFF by default** to simulate a replacement scenario.

## Robot Specs

| Property | Value |
|----------|-------|
| Chassis | 0.9 x 0.64 x 0.19m, 70kg |
| Drive | Differential, wheel_radius=0.1m, wheel_sep=0.6m |
| Wheels | middle_left_wheel_joint, middle_right_wheel_joint |
| Nav2 | robot_radius=0.22m, max_vel=0.26m/s |

## Demo Scenario: LiDAR Replacement

```bash
# 1. Check LiDAR status (should be DOWN)
bash ~/scripts/lidar_diagnostic.sh

# 2. Start LiDAR (simulates post-replacement)
bash ~/scripts/start_lidar.sh

# 3. Verify LiDAR is running
bash ~/scripts/lidar_diagnostic.sh
```

## Scripts

| Script | Purpose |
|--------|---------|
| `scripts/lidar_diagnostic.sh` | Check 2D LiDAR node, topic, and rate |
| `scripts/start_lidar.sh` | Start the 2D LiDAR node after replacement |

## Network

- SSH: `localhost:2222` (user: `ubuntu`, password: `ubuntu`)
- ROS2 DDS: Cyclone DDS, domain 0

## Documentation (docs/)

Robot documentation structured for upload to the Farhand Relay platform as resources.

```
docs/
├── robot/
│   └── Technical_Reference.md              # Complete robot specs, sensors, ROS2 topics
├── sensors/
│   ├── 2d-lidar/
│   │   └── 2D_LiDAR_Reference.md          # 2D LiDAR specifications
│   ├── depth-camera/
│   │   └── Depth_Camera_Reference.md       # Depth camera specifications
│   └── stereo-camera/
│       └── Stereo_Camera_Reference.md      # Stereo camera specifications
├── procedures/
│   ├── SOP-HW-007_LiDAR_Replacement.md    # Field replacement SOP
│   └── SOP-NW-001_WiFi_Reconnection.md    # WiFi reconnection procedure
└── diagnostics/
    └── SCR-DIAG-003_LiDAR_Network_Diagnostic.md  # Automated LiDAR diagnostic script
```

Upload the entire `docs/` folder to HQ > Resources on the Farhand Relay platform.


## File Structure

```
farhand-amr-demo/
├── Dockerfile              # ROS2 Humble + SSH server
├── docker-compose.yml      # Container config (port 2222)
├── entrypoint.sh           # Starts SSH + base nodes (no LiDAR)
├── nodes/
│   ├── lidar_2d_node.py    # 2D LiDAR (361 samples @ 30Hz)
│   ├── depth_camera_node.py # Depth camera (640x480 @ 30Hz)
│   ├── stereo_camera_node.py # Stereo camera (1024x1024 @ 10Hz)
│   ├── drive_node.py       # Diff drive (odom + joints @ 50Hz)
│   ├── imu_node.py         # IMU (5Hz)
│   └── battery_node.py     # Battery state (1Hz)
├── scripts/
│   ├── lidar_diagnostic.sh # Check 2D LiDAR health
│   └── start_lidar.sh      # Start LiDAR after replacement
└── docs/                   # Upload to Farhand Field platform
    ├── robot/              # Robot reference docs
    ├── sensors/            # Sensor reference docs
    ├── procedures/         # SOPs
    └── diagnostics/        # Diagnostic scripts
```
