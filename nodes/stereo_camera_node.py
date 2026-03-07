#!/usr/bin/env python3
"""Simulated stereo camera — publishes left/right Image + CameraInfo at 10Hz.

Fault mode: STEREO_DESYNC=1 → right camera drops to 3Hz while left stays at 10Hz.
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
            f.write(f'[{ts}] STEREO: {msg}\n')
    except Exception:
        pass


class StereoCameraNode(Node):
    BASELINE = 0.06

    def __init__(self):
        super().__init__('stereo_camera_node')
        self.left_img_pub = self.create_publisher(Image, '/bcr_bot/stereo/left/image_raw', 10)
        self.left_info_pub = self.create_publisher(CameraInfo, '/bcr_bot/stereo/left/camera_info', 10)
        self.right_img_pub = self.create_publisher(Image, '/bcr_bot/stereo/right/image_raw', 10)
        self.right_info_pub = self.create_publisher(CameraInfo, '/bcr_bot/stereo/right/camera_info', 10)

        self.desync = os.environ.get('STEREO_DESYNC') == '1'
        self.frame = 0

        # Left always at 10Hz
        self.left_timer = self.create_timer(1.0 / 10.0, self.publish_left)
        # Right at 3Hz if desync, else 10Hz
        right_rate = 3.0 if self.desync else 10.0
        self.right_timer = self.create_timer(1.0 / right_rate, self.publish_right)

        mode = f'DESYNC (L=10Hz, R={right_rate}Hz)' if self.desync else 'normal (10Hz)'
        self.get_logger().info(f'Stereo camera started ({mode}, 1024x1024, 0.06m baseline)')
        if self.desync:
            log(f'WARNING — Left/right rate mismatch: L=10Hz R={right_rate}Hz')
            log('Stereo depth estimation will be unreliable')

    def _make_image(self, stamp, frame_id, w, h):
        img = np.zeros((h, w, 3), dtype=np.uint8)
        img[:, :, 0] = np.arange(w, dtype=np.uint8)[:w]
        img[:, :, 1] = np.arange(h, dtype=np.uint8).reshape(-1, 1)[:h]
        img[:, :, 2] = self.frame % 256

        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.height = h
        msg.width = w
        msg.encoding = 'rgb8'
        msg.is_bigendian = False
        msg.step = w * 3
        msg.data = img.tobytes()
        return msg

    def _make_info(self, header, w, h, tx=0.0):
        hfov_rad = math.radians(60.0)
        fx = w / (2.0 * math.tan(hfov_rad / 2.0))
        fy = fx

        info = CameraInfo()
        info.header = header
        info.height = h
        info.width = w
        info.distortion_model = 'plumb_bob'
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = [fx, 0.0, w / 2.0, 0.0, fy, h / 2.0, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, w / 2.0, tx, 0.0, fy, h / 2.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info

    def publish_left(self):
        stamp = self.get_clock().now().to_msg()
        w, h = 1024, 1024
        left_img = self._make_image(stamp, 'stereo_left_link', w, h)
        left_info = self._make_info(left_img.header, w, h, tx=0.0)
        self.left_img_pub.publish(left_img)
        self.left_info_pub.publish(left_info)
        self.frame += 1

    def publish_right(self):
        stamp = self.get_clock().now().to_msg()
        w, h = 1024, 1024
        hfov_rad = math.radians(60.0)
        fx = w / (2.0 * math.tan(hfov_rad / 2.0))
        right_img = self._make_image(stamp, 'stereo_right_link', w, h)
        right_info = self._make_info(right_img.header, w, h, tx=-fx * self.BASELINE)
        self.right_img_pub.publish(right_img)
        self.right_info_pub.publish(right_info)


def main():
    rclpy.init()
    node = StereoCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
