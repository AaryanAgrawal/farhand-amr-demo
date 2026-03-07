#!/usr/bin/env python3
"""Simulated depth camera — publishes Image + CameraInfo at 30Hz.

Fault mode: CAMERA_FAULT=1 → all-zero images (USB disconnect).
"""

import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import numpy as np
import math
from datetime import datetime

LOG_PATH = os.path.expanduser('~/logs/sensor_health.log')


def log(msg):
    ts = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    try:
        with open(LOG_PATH, 'a') as f:
            f.write(f'[{ts}] DEPTH_CAMERA: {msg}\n')
    except Exception:
        pass


class DepthCameraNode(Node):
    def __init__(self):
        super().__init__('depth_camera_node')
        self.img_pub = self.create_publisher(Image, '/bcr_bot/camera/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/bcr_bot/camera/camera_info', 10)
        self.timer = self.create_timer(1.0 / 30.0, self.publish)  # 30 Hz
        self.frame = 0
        self.fault = os.environ.get('CAMERA_FAULT') == '1'

        mode = 'FAULT (zero data)' if self.fault else 'normal'
        self.get_logger().info(f'Depth camera started (30Hz, 640x480, {mode})')
        if self.fault:
            log('ERROR — Frame data all zeros. Possible USB disconnect.')
            log('Driver still running but no valid frames received.')

    def publish(self):
        stamp = self.get_clock().now().to_msg()
        w, h = 640, 480

        if self.fault:
            img = np.zeros((h, w, 3), dtype=np.uint8)
        else:
            img = np.zeros((h, w, 3), dtype=np.uint8)
            img[:, :, 0] = np.arange(w, dtype=np.uint8)
            img[:, :, 1] = np.arange(h, dtype=np.uint8).reshape(-1, 1)
            img[:, :, 2] = self.frame % 256

        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = 'depth_camera_link'
        msg.height = h
        msg.width = w
        msg.encoding = 'rgb8'
        msg.is_bigendian = False
        msg.step = w * 3
        msg.data = img.tobytes()
        self.img_pub.publish(msg)

        hfov_rad = math.radians(60.0)
        fx = w / (2.0 * math.tan(hfov_rad / 2.0))
        fy = fx

        info = CameraInfo()
        info.header = msg.header
        info.height = h
        info.width = w
        info.distortion_model = 'plumb_bob'
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = [fx, 0.0, w / 2.0, 0.0, fy, h / 2.0, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, w / 2.0, 0.0, 0.0, fy, h / 2.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.info_pub.publish(info)

        self.frame += 1


def main():
    rclpy.init()
    node = DepthCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
