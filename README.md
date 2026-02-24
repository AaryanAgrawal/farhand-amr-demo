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

Verify running topics:
```bash
ros2 topic list
```

Note: `/livox/lidar` is NOT listed — the LiDAR is not running yet.

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

## Documentation (docs/)

The `docs/` folder contains all robot documentation, structured for upload to the Farhand Field platform as resources.

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

## Platform Demo Flow

### Prerequisites

- Demo site: https://field.farhand.live
- Login: `admin@farhand.live` / `Admin123!` (system_admin)
- FDE login: `fde@farhand.live` / `Fde12345!` (field technician)

### Steps

1. **Upload docs** — Go to HQ > AI Assistant. Upload files from `docs/` via the paperclip button.
2. **Generate SOP** — Ask: "Create an SOP for monthly preventive maintenance of the Livox Mid-360 LiDAR."
3. **Generate Script** — Ask: "Create a diagnostic script for the Livox Mid-360 LiDAR that checks network connectivity, ROS2 node status, and point cloud rate."
4. **Create Site** — Go to HQ > Sites. Create "SF Warehouse" (San Francisco, CA 94107, robot IP 192.168.1.50, user ubuntu).
5. **Create Work Order** — Go to HQ > Work Orders. Description: "Replace broken LiDAR sensor on BCR-001." Parts: LIV-MID360-01, CBL-ETH-3M, MNT-LIDAR-360.
6. **Assign to FDE** — Assign the work order to `fde@farhand.live`.
7. **FDE View** — Log in as FDE to see the assigned work order with job plan, parts, and site info.

### Reset Demo Data (Optional)

```bash
TOKEN=$(curl -s https://field.farhand.live/api/field/auth/login \
  -H 'Content-Type: application/json' \
  -d '{"email":"admin@farhand.live","password":"Admin123!"}' | jq -r '.token')

curl -s https://field.farhand.live/api/field/admin/reset-demo-data \
  -H "Authorization: Bearer $TOKEN" \
  -X POST | jq
```

## For Farhand Field CLI

```bash
farhand-field --robot-ip=localhost --robot-user=ubuntu
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
