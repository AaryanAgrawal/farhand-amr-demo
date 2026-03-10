#!/usr/bin/env python3
"""Simulated depth camera — publishes Image + CameraInfo.

Fault: fault.usb_disconnect → all-zero images.
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
            f.write(f'[{ts}] DEPTH_CAMERA: {msg}\n')
    except Exception:
        pass


class DepthCameraNode(Node):
    def __init__(self):
        super().__init__('depth_camera_node')

        self.declare_parameter('rate_hz', 30.0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('hfov_deg', 60.0)
        self.declare_parameter('fault.usb_disconnect', False)

        self.w = self.get_parameter('width').value
        self.h = self.get_parameter('height').value
        self.hfov_deg = self.get_parameter('hfov_deg').value
        rate = self.get_parameter('rate_hz').value
        self.fault = self.get_parameter('fault.usb_disconnect').value

        self.frame = 0
        self.img_pub = self.create_publisher(Image, 'camera/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, 'camera/camera_info', 10)
        self.timer = self.create_timer(1.0 / rate, self.publish)

        self.add_on_set_parameters_callback(self._on_param_change)

        mode = 'FAULT (zero data)' if self.fault else 'normal'
        self.get_logger().info(f'Depth camera started ({rate}Hz, {self.w}x{self.h}, {mode})')
        if self.fault:
            log('ERROR — Frame data all zeros. Possible USB disconnect.')

    def _on_param_change(self, params):
        for p in params:
            if p.name == 'fault.usb_disconnect':
                self.fault = p.value
                if self.fault:
                    log('FAULT INJECTED — USB disconnect, zero frames')
                else:
                    log('Fault cleared — normal frames restored')
        return SetParametersResult(successful=True)

    def publish(self):
        stamp = self.get_clock().now().to_msg()
        w, h = self.w, self.h

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

        hfov_rad = math.radians(self.hfov_deg)
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
