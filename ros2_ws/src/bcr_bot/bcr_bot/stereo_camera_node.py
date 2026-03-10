#!/usr/bin/env python3
"""Simulated stereo camera — publishes left/right Image + CameraInfo.

Fault: fault.desync → right camera drops to 3Hz while left stays at nominal rate.
"""

import os
import math
from datetime import datetime

import numpy as np
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Image, CameraInfo

LOG_PATH = os.path.expanduser('~/logs/sensor_health.log')


def log(msg):
    ts = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    try:
        with open(LOG_PATH, 'a') as f:
            f.write(f'[{ts}] STEREO: {msg}\n')
    except Exception:
        pass


class StereoCameraNode(Node):
    def __init__(self):
        super().__init__('stereo_camera_node')

        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('width', 1024)
        self.declare_parameter('height', 1024)
        self.declare_parameter('baseline', 0.06)
        self.declare_parameter('hfov_deg', 60.0)
        self.declare_parameter('fault.desync', False)

        self.nominal_rate = self.get_parameter('rate_hz').value
        self.w = self.get_parameter('width').value
        self.h = self.get_parameter('height').value
        self.baseline = self.get_parameter('baseline').value
        self.hfov_deg = self.get_parameter('hfov_deg').value
        self.desync = self.get_parameter('fault.desync').value

        self.frame = 0

        self.left_img_pub = self.create_publisher(Image, 'stereo/left/image_raw', 10)
        self.left_info_pub = self.create_publisher(CameraInfo, 'stereo/left/camera_info', 10)
        self.right_img_pub = self.create_publisher(Image, 'stereo/right/image_raw', 10)
        self.right_info_pub = self.create_publisher(CameraInfo, 'stereo/right/camera_info', 10)

        self.left_timer = self.create_timer(1.0 / self.nominal_rate, self.publish_left)
        right_rate = 3.0 if self.desync else self.nominal_rate
        self.right_timer = self.create_timer(1.0 / right_rate, self.publish_right)

        self.add_on_set_parameters_callback(self._on_param_change)

        mode = f'DESYNC (L={self.nominal_rate}Hz, R={right_rate}Hz)' if self.desync else f'normal ({self.nominal_rate}Hz)'
        self.get_logger().info(f'Stereo camera started ({mode}, {self.w}x{self.h}, {self.baseline}m baseline)')
        if self.desync:
            log(f'WARNING — Left/right rate mismatch: L={self.nominal_rate}Hz R={right_rate}Hz')

    def _on_param_change(self, params):
        for p in params:
            if p.name == 'fault.desync':
                self.desync = p.value
                right_rate = 3.0 if self.desync else self.nominal_rate
                self.right_timer.cancel()
                self.right_timer = self.create_timer(1.0 / right_rate, self.publish_right)
                if self.desync:
                    log(f'FAULT INJECTED — right camera rate dropped to {right_rate}Hz')
                else:
                    log('Fault cleared — stereo sync restored')
        return SetParametersResult(successful=True)

    def _make_image(self, stamp, frame_id):
        w, h = self.w, self.h
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

    def _make_info(self, header, tx=0.0):
        w, h = self.w, self.h
        hfov_rad = math.radians(self.hfov_deg)
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
        left_img = self._make_image(stamp, 'stereo_left_link')
        left_info = self._make_info(left_img.header, tx=0.0)
        self.left_img_pub.publish(left_img)
        self.left_info_pub.publish(left_info)
        self.frame += 1

    def publish_right(self):
        stamp = self.get_clock().now().to_msg()
        hfov_rad = math.radians(self.hfov_deg)
        fx = self.w / (2.0 * math.tan(hfov_rad / 2.0))
        right_img = self._make_image(stamp, 'stereo_right_link')
        right_info = self._make_info(right_img.header, tx=-fx * self.baseline)
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
