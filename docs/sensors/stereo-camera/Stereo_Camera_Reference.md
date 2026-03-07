# Stereo Camera — Technical Reference

## Overview
The BCR Bot includes a front-facing stereo camera pair for depth estimation and 3D perception. Each camera provides 1024x1024 images with a 0.06m baseline between left and right sensors.

## Specifications
| Parameter | Value |
|-----------|-------|
| Resolution (per eye) | 1024 x 1024 |
| Frame rate | 10 Hz |
| Encoding | RGB8 |
| Horizontal FOV | 60 deg |
| Baseline | 0.06 m |

## ROS2 Interface
| Topic | Message Type | Rate |
|-------|-------------|------|
| `/bcr_bot/stereo/left/image_raw` | sensor_msgs/Image | 10 Hz |
| `/bcr_bot/stereo/right/image_raw` | sensor_msgs/Image | 10 Hz |
| `/bcr_bot/stereo/left/camera_info` | sensor_msgs/CameraInfo | 10 Hz |
| `/bcr_bot/stereo/right/camera_info` | sensor_msgs/CameraInfo | 10 Hz |

### CameraInfo Intrinsics
- Distortion model: plumb_bob (no distortion in simulation)
- Focal length: fx = fy = w / (2 * tan(HFOV/2)) = ~886.81 px
- Principal point: (512, 512)
- Left frame: `stereo_left_link`
- Right frame: `stereo_right_link`
- Right camera P matrix Tx = -fx * baseline (encodes the stereo baseline)

## Stereo Geometry
The left camera is the reference camera. The right camera is offset by the baseline distance (0.06m) along the x-axis. The projection matrix P for the right camera encodes this offset as Tx = -fx * 0.06.

## Mounting
- Position: front-facing, slightly above depth camera
- Baseline axis: horizontal (left-right)

## Diagnostics
Check if the stereo camera node is running:
```bash
ros2 node list | grep stereo_camera
```

Check topic rate:
```bash
ros2 topic hz /bcr_bot/stereo/left/image_raw
```
Expected: ~10 Hz

## Common Issues
| Symptom | Likely Cause | Resolution |
|---------|-------------|------------|
| Only one camera publishing | Node partially started | Restart stereo_camera_node |
| Disparity map empty | Insufficient texture in scene | Verify both cameras produce different images |
| Rate < 8 Hz | High CPU/bandwidth usage | 1024x1024 images are large; check resources |

## Usage Notes
- Stereo matching can be performed using `stereo_image_proc` from the `image_pipeline` package
- Depth range is limited by baseline: at 0.06m baseline, reliable depth up to ~10m
- Higher resolution (1024x1024) provides better depth accuracy at the cost of bandwidth
