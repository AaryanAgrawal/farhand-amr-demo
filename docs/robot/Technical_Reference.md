# BCR Bot — Technical Reference

## Overview
- Type: Differential drive AMR
- Source: [blackcoffeerobotics/bcr_bot](https://github.com/blackcoffeerobotics/bcr_bot)
- Use case: Indoor warehouse logistics, general-purpose AMR
- Chassis: 0.9 x 0.64 x 0.19m, 70kg
- Max speed: 0.26 m/s (Nav2 configured)
- Robot radius: 0.22m (Nav2 footprint)

## Hardware Components
| Component | Type | Location |
|-----------|------|----------|
| 2D LiDAR | 360 deg laser scanner | Top-mounted, center |
| Depth Camera | RGB-D (640x480) | Front-facing |
| Stereo Camera | Dual 1024x1024 | Front-facing |
| IMU | 6-axis IMU | Inside chassis, at imu_frame |
| Motors | 2x DC motors | Middle left/right wheels |
| Battery | 3S LiPo, 5Ah | Bottom compartment |

## Sensors

### 2D LiDAR
- Samples: 361 per scan
- Rate: 30Hz
- Range: 0.55m – 16m
- FOV: 360 deg horizontal
- ROS2 topic: `/bcr_bot/scan` (sensor_msgs/LaserScan)
- Frame: `lidar_link`
- Health check: `ros2 topic hz /bcr_bot/scan` (expect ~30Hz)

### Depth Camera
- Resolution: 640x480 RGB8, 30Hz
- HFOV: 60 deg
- Clipping range: 0.05m – 8m
- ROS2 topics: `/bcr_bot/camera/image_raw` (Image), `/bcr_bot/camera/camera_info` (CameraInfo)
- Frame: `depth_camera_link`
- Health check: `ros2 topic hz /bcr_bot/camera/image_raw` (expect ~30Hz)

### Stereo Camera
- Resolution: 1024x1024 per eye, 10Hz
- HFOV: 60 deg
- Baseline: 0.06m
- ROS2 topics:
  - `/bcr_bot/stereo/left/image_raw` (Image)
  - `/bcr_bot/stereo/right/image_raw` (Image)
  - `/bcr_bot/stereo/left/camera_info` (CameraInfo)
  - `/bcr_bot/stereo/right/camera_info` (CameraInfo)
- Frames: `stereo_left_link`, `stereo_right_link`
- Health check: `ros2 topic hz /bcr_bot/stereo/left/image_raw` (expect ~10Hz)

### IMU
- Rate: 5Hz
- ROS2 topic: `/bcr_bot/imu` (sensor_msgs/Imu)
- Data: orientation, angular velocity, linear acceleration
- Frame: `imu_frame`

## Network
- ROS2 middleware: Cyclone DDS, domain 0
- SSH default user: ubuntu
- Note: Actual robot IP, SSH port are per-instance (from work order context)

## ROS2 Topics
| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/bcr_bot/scan` | LaserScan | 30Hz | 2D LiDAR scan |
| `/bcr_bot/camera/image_raw` | Image | 30Hz | Depth camera RGB |
| `/bcr_bot/camera/camera_info` | CameraInfo | 30Hz | Depth camera intrinsics |
| `/bcr_bot/stereo/left/image_raw` | Image | 10Hz | Stereo left |
| `/bcr_bot/stereo/right/image_raw` | Image | 10Hz | Stereo right |
| `/bcr_bot/stereo/left/camera_info` | CameraInfo | 10Hz | Stereo left intrinsics |
| `/bcr_bot/stereo/right/camera_info` | CameraInfo | 10Hz | Stereo right intrinsics |
| `/bcr_bot/cmd_vel` | Twist | — | Velocity commands |
| `/bcr_bot/odom` | Odometry | 50Hz | Wheel odometry |
| `/bcr_bot/joint_states` | JointState | 50Hz | middle_left/right_wheel_joint |
| `/bcr_bot/imu` | Imu | 5Hz | IMU data |
| `/bcr_bot/battery_state` | BatteryState | 1Hz | Voltage, current, percentage |

## Software Stack
- OS: Ubuntu 22.04 LTS
- ROS2: Humble Hawksbill
- DDS: Cyclone DDS
- Navigation: Nav2 (navigation2)
- SLAM: slam_toolbox

## Drive Configuration
- Type: Differential drive
- Wheel separation: 0.6m
- Wheel radius: 0.1m
- Joints: `middle_left_wheel_joint`, `middle_right_wheel_joint`

## Nav2 Parameters
- Robot radius: 0.22m
- Max velocity: 0.26 m/s
- Controller: DWB local planner (default)
- Planner: NavFn global planner (default)

## Common Issues
- LiDAR not publishing: check `two_d_lidar_node` is running, verify `/bcr_bot/scan` topic
- Camera black: verify depth_camera_node process, check `/bcr_bot/camera/image_raw`
- Navigation drift: check `/bcr_bot/scan` rate, verify SLAM map, check wheel encoders via `/bcr_bot/odom`
- Cannot SSH: verify robot is on correct subnet, check SSH port, verify credentials
- Battery low: check `/bcr_bot/battery_state` for voltage <10.5V, replace or charge

## Maintenance
- LiDAR lens: clean weekly with microfiber cloth
- Wheels: inspect monthly for wear and debris
- Firmware: coordinate with HQ before updating
