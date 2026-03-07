# 2D LiDAR Sensor — Technical Reference

## Overview
The BCR Bot uses a 2D laser scanner for obstacle detection, SLAM mapping, and autonomous navigation. The sensor provides 360-degree planar coverage with 361 range measurements per scan.

## Specifications
| Parameter | Value |
|-----------|-------|
| Type | 2D laser scanner (LaserScan) |
| Samples per scan | 361 |
| Scan rate | 30 Hz |
| Range (min) | 0.55 m |
| Range (max) | 16 m |
| Angular range | 360 deg |
| Angular resolution | ~1 deg (360/361) |

## ROS2 Interface
| Topic | Message Type | Rate |
|-------|-------------|------|
| `/bcr_bot/scan` | sensor_msgs/LaserScan | 30 Hz |

### LaserScan Message Fields
| Field | Value |
|-------|-------|
| angle_min | -pi (-180 deg) |
| angle_max | +pi (+180 deg) |
| angle_increment | 2*pi / 360 |
| range_min | 0.55 m |
| range_max | 16.0 m |
| frame_id | lidar_link |

## Mounting
- Position: top-mounted, center of chassis
- Height: above all other sensors for unobstructed 360-degree view
- Orientation: forward-facing (0 deg = robot front)

## Diagnostics
Check if the LiDAR node is running:
```bash
ros2 node list | grep two_d_lidar
```

Check topic rate:
```bash
ros2 topic hz /bcr_bot/scan
```
Expected: ~30 Hz

View a single scan:
```bash
ros2 topic echo /bcr_bot/scan --once
```

## Common Issues
| Symptom | Likely Cause | Resolution |
|---------|-------------|------------|
| No `/bcr_bot/scan` topic | Node not started | Run `bash ~/scripts/start_lidar.sh` |
| Rate < 20 Hz | CPU overload | Check system load, reduce other node rates |
| All ranges = inf | Sensor obstruction | Clean lens, check mounting |
| Partial scan (gaps) | Mechanical obstruction | Verify 360-degree clearance around sensor |

## Integration Notes
- Nav2 costmap uses this sensor for obstacle detection
- slam_toolbox uses this sensor for map building
- The sensor is intentionally started OFF in the demo to simulate a replacement scenario
