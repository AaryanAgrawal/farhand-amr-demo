# BCR-001 — Technical Reference

## Overview
- Type: Differential drive AMR (Box Carrier Robot)
- Use case: Indoor warehouse logistics
- Compute: NVIDIA Jetson Orin Nano
- Dimensions: 500 x 400 x 350mm, 12kg
- Max speed: 1.0 m/s

## Hardware Components
| Component | Model | Location | Part Number |
|-----------|-------|----------|-------------|
| LiDAR | Livox Mid-360 | Top-mounted, center | LIV-MID360-01 |
| Camera | USB 2.0 (640x480 RGB) | Front-facing | — |
| IMU | BNO055 | Inside chassis | — |
| Motors | 2x brushless DC | Left/right wheels | — |
| Battery | 3S LiPo, 5Ah | Bottom compartment | — |

## Sensors

### Livox Mid-360 LiDAR
- Point rate: 200,000 pts/s, frame rate: 10Hz
- Range: 0.1m - 40m (10% reflectivity), 70m (80% reflectivity)
- FOV: 360° horizontal, -7° to 52° vertical
- Default IP: 192.168.1.1xx (actual IP is per-robot instance config)
- Ports: 56100 (command), 56200 (data), 56300 (IMU) — Livox SDK protocol
- Mounting: 4x M3 socket head cap screws, torque 0.5 Nm, arrow points forward
- Driver service: livox_ros2_driver
- ROS2 topic: /livox/lidar (sensor_msgs/PointCloud2, 10Hz)
- Built-in IMU: ICM40609, topic /livox/imu
- Config path: /opt/livox/config/MID360_config.json
- LED status: Solid Red = powering on, Blinking Green = awaiting connection, Solid Green = streaming
- Health check: ping <lidar_ip>, ros2 topic hz /livox/lidar (expect ~10Hz)

### USB Camera
- Resolution: 640x480 RGB8, 30Hz
- Connection: USB 2.0, front-facing
- ROS2 topics: /camera/image_raw (Image), /camera/camera_info (CameraInfo)
- Health check: ls /dev/video*, ros2 topic hz /camera/image_raw (expect ~30Hz)

### IMU (BNO055)
- Rate: 100Hz
- ROS2 topic: /imu/data (sensor_msgs/Imu)
- Data: orientation, angular velocity, linear acceleration

## Network (type-level defaults)
- ROS2 middleware: Cyclone DDS, domain 0
- LiDAR default subnet: 192.168.1.x
- SSH default user: ubuntu
- Note: Actual robot IP, SSH port, and LiDAR IP are per-instance (from work order context)

## ROS2 Topics
| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| /livox/lidar | PointCloud2 | 10Hz | LiDAR point cloud |
| /camera/image_raw | Image | 30Hz | RGB camera |
| /camera/camera_info | CameraInfo | 30Hz | Camera intrinsics |
| /cmd_vel | Twist | — | Velocity commands |
| /odom | Odometry | 50Hz | Wheel odometry |
| /joint_states | JointState | 50Hz | left_wheel_joint, right_wheel_joint |
| /imu/data | Imu | 100Hz | IMU data |
| /battery_state | BatteryState | 1Hz | Voltage, current, percentage |

## Software Stack
- OS: Ubuntu 22.04 LTS
- ROS2: Humble Hawksbill
- DDS: Cyclone DDS
- Navigation: Nav2 (navigation2)
- SLAM: slam_toolbox
- Visualization: Foxglove Bridge

## Drive Configuration
- Wheel separation: 340mm
- Wheel radius: 50mm
- Joints: left_wheel_joint, right_wheel_joint

## Common Issues
- LiDAR not publishing: check Ethernet to LiDAR, ping <lidar_ip>, restart livox_ros2_driver, check LED status
- Camera black: check USB connection, ls /dev/video*, restart camera node
- Navigation drift: check /livox/lidar rate, verify SLAM map, check wheel encoders via /odom
- Cannot SSH: verify robot is on correct subnet, check SSH port, verify credentials
- Battery low: check /battery_state for voltage <10.5V, replace or charge

## Maintenance
- LiDAR lens: clean weekly with microfiber cloth
- Wheels: inspect monthly for wear and debris
- Firmware: coordinate with HQ before updating
- Tools required: 3mm hex key (LiDAR M3 screws), Phillips PH2 (chassis panels)
