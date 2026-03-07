# Depth Camera — Technical Reference

## Overview
The BCR Bot uses a front-facing depth camera for RGB imaging and depth perception. The camera provides color images at 640x480 resolution and depth data for obstacle avoidance and object detection.

## Specifications
| Parameter | Value |
|-----------|-------|
| Resolution | 640 x 480 |
| Frame rate | 30 Hz |
| Encoding | RGB8 |
| Horizontal FOV | 60 deg |
| Clip near | 0.05 m |
| Clip far | 8.0 m |

## ROS2 Interface
| Topic | Message Type | Rate |
|-------|-------------|------|
| `/bcr_bot/camera/image_raw` | sensor_msgs/Image | 30 Hz |
| `/bcr_bot/camera/camera_info` | sensor_msgs/CameraInfo | 30 Hz |

### CameraInfo Intrinsics
- Distortion model: plumb_bob (no distortion in simulation)
- Focal length: fx = fy = w / (2 * tan(HFOV/2)) = ~554.26 px
- Principal point: (320, 240)
- Frame: `depth_camera_link`

## Mounting
- Position: front-facing, center of robot
- Use case: obstacle detection, visual odometry, object recognition

## Diagnostics
Check if the depth camera node is running:
```bash
ros2 node list | grep depth_camera
```

Check topic rate:
```bash
ros2 topic hz /bcr_bot/camera/image_raw
```
Expected: ~30 Hz

## Common Issues
| Symptom | Likely Cause | Resolution |
|---------|-------------|------------|
| No image topic | Node not started | Check entrypoint logs |
| Black/uniform image | Simulation rendering issue | Restart node |
| Low frame rate | CPU/bandwidth limitation | Check system resources |
