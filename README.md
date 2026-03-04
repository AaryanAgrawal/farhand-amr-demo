# Farhand AMR Test Bench (BCR-001)

Dockerized ROS2 robot simulator for testing the [Farhand Field](https://field.farhand.live) platform and CLI agent. Emulates a BCR-001 AMR with cameras, IMU, differential drive, battery, and a Livox Mid-360 LiDAR that starts OFF to simulate a replacement scenario.

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

Note: `/livox/lidar` is NOT listed — the LiDAR is intentionally off.

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
| Livox Mid-360 LiDAR | `/livox/lidar` | 10Hz | `lidar_node.py` |
| USB Camera | `/camera/image_raw` | 30Hz | `camera_node.py` |
| Differential Drive | `/odom`, `/joint_states` | 50Hz | `drive_node.py` |
| IMU (BNO055) | `/imu/data` | 100Hz | `imu_node.py` |
| Battery (3S LiPo) | `/battery_state` | 1Hz | `battery_node.py` |

**The LiDAR starts OFF by default** to simulate a replacement scenario.

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
| `scripts/lidar_diagnostic.sh` | Check LiDAR node, topic, and rate |
| `scripts/start_lidar.sh` | Start the LiDAR node after replacement |

## Documentation (docs/)

Robot documentation structured for upload to the Farhand Field platform as resources.

```
docs/
├── robot/
│   └── BCR-001_Technical_Reference.md       # Complete robot specs, sensors, ROS2 topics
├── sensors/
│   └── livox-mid-360/
│       ├── Livox_Mid-360_Datasheet.md       # Technical specifications
│       ├── Livox_Mid-360_Integration_Guide.md  # SDK installation and config
│       ├── Livox_Mid-360_User_Manual.pdf    # Official vendor manual
│       └── Livox_Mid-360_Quick_Start.pdf    # Official vendor quick start
├── procedures/
│   └── SOP-HW-007_LiDAR_Replacement.md     # 12-step field replacement SOP
└── diagnostics/
    └── SCR-DIAG-003_LiDAR_Network_Diagnostic.md  # Automated LiDAR diagnostic script
```

Upload the entire `docs/` folder to HQ > Resources on the Farhand Field platform.

## CLI Agent

```bash
farhand-field --robot-ip=localhost --robot-user=ubuntu
```

Or via Tailscale:
```bash
farhand-field --robot-ip=100.84.147.55 --robot-user=ubuntu
```

The CLI agent can SSH in and run diagnostics, start the LiDAR, verify the fix — all through natural language.

## File Structure

```
farhand-amr-demo/
├── Dockerfile              # ROS2 Humble + SSH server
├── docker-compose.yml      # Container config (port 2222)
├── entrypoint.sh           # Starts SSH + base nodes (no LiDAR)
├── nodes/
│   ├── lidar_node.py       # Livox Mid-360 (200k pts @ 10Hz)
│   ├── camera_node.py      # USB camera (640x480 @ 30Hz)
│   ├── drive_node.py       # Diff drive (odom + joints @ 50Hz)
│   ├── imu_node.py         # IMU (100Hz)
│   └── battery_node.py     # Battery state (1Hz)
├── scripts/
│   ├── lidar_diagnostic.sh # Check LiDAR health
│   └── start_lidar.sh      # Start LiDAR after replacement
└── docs/                   # Upload to Farhand Field platform
    ├── robot/              # Robot reference docs
    ├── sensors/            # Sensor vendor docs
    ├── procedures/         # SOPs
    └── diagnostics/        # Diagnostic scripts
```
