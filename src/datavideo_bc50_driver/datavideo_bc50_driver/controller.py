#!/usr/bin/env python3
"""
ROS 2 driver for the DataVideo BC-50 zoom camera.
Publishes image + CameraInfo and offers zoom control.
"""

import base64
import os
import time
from datetime import datetime
from threading import Lock

import threading
from collections import deque
from rclpy.qos import QoSProfile, ReliabilityPolicy

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String, Int16
from std_srvs.srv import Trigger
from sensor_msgs.srv import SetCameraInfo

import urllib3
import yaml


class ZoomController(Node):
    def __init__(self):
        super().__init__('bc50_controller')

        # ---------------- Parameters ----------------
        self.declare_parameter('camera_hostname', '192.168.100.99')
        self.declare_parameter('username', 'admin')
        self.declare_parameter('password', 'admin')
        self.declare_parameter('camera_info_url', '')
        self.declare_parameter('frame_id', 'camera_frame')
        self.declare_parameter('max_zoom_level', 36)
        self.declare_parameter('publish_rate_hz', 30.0)

        self.camera_hostname = self.get_parameter('camera_hostname').get_parameter_value().string_value
        username = self.get_parameter('username').get_parameter_value().string_value
        password = self.get_parameter('password').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.zoom_level = self.get_parameter('max_zoom_level').get_parameter_value().integer_value  # start zoomed out
        self.publish_period = 1.0 / self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        self.max_zoom_level = self.zoom_level

        self.auth_header = base64.b64encode(f'{username}:{password}'.encode()).decode()
        self.headers = {
            'User-Agent': 'bc50-driver',
            'Authorization': f'Basic {self.auth_header}',
            'Connection': 'keep-alive',
            'Accept': '*/*',
        }

        # ---------------- Internal helpers ----------------
        self.bridge = CvBridge()
        self.zoom_lock = Lock()
        self.camera_lock = Lock()
        self.rtsp_url = f"rtsp://{self.camera_hostname}:554"
        self.cap = None
        self.http = urllib3.PoolManager(retries=urllib3.Retry(3, backoff_factor=0.1))

        # Capture directory
        self.save_dir = os.path.expanduser('/home/ali/tests/zoom_cam/camera_captures')
        os.makedirs(self.save_dir, exist_ok=True)

        # # Ring buffer for latest frame (size 1 keeps only the newest)
        # self.frame_buffer = deque(maxlen=1)

        # ---------------- Publishers/Subscribers/Services ----------------
        reliable_qos = QoSProfile(depth=10,
                                reliability=ReliabilityPolicy.RELIABLE,
                                durability=qos_profile_sensor_data.durability)
        self.image_pub = self.create_publisher(Image,
                                            'camera/image_raw',
                                            reliable_qos)
        self.camera_info_pub = self.create_publisher(CameraInfo, 'camera/camera_info', 10)

        self.create_subscription(String, 'camera_command', self.command_callback, 10)
        self.create_subscription(Int16, 'set_level_command', self.level_callback, 10)

        self.create_service(Trigger, 'capture_image', self.capture_service_cb)
        self.create_service(SetCameraInfo, 'camera/set_camera_info', self.set_camera_info_cb)

        # ------------- CameraInfo -------------
        self.camera_info = CameraInfo()
        self.load_camera_info()

        # ------------- First RTSP connect (blocking, one-shot) -------------
        self.initialize_camera()                       # <-- move *before* the thread

        # ------------- Background grab + publish timer -------------
        self.frame_buffer = deque(maxlen=1)
        self.grab_thread = threading.Thread(target=self.grab_loop,
                                            daemon=True)
        self.grab_thread.start()

        self.publish_timer = self.create_timer(self.publish_period,
                                               self.publish_image)

        # ------------- Startup zoom -------------
        self.zoom_out_all()
        self.get_logger().info('BC-50 controller initialised')

    # =========================================================
    # Camera initialisation & calibration
    # =========================================================
    def initialize_camera(self):
        try:
            with self.camera_lock:
                if self.cap:
                    self.cap.release()
                self.cap = cv2.VideoCapture(self.rtsp_url, cv2.CAP_FFMPEG)
                self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

                if not self.cap.isOpened():
                    raise RuntimeError('RTSP open failed')
                # Ensure resolution
                # self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
                # self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
            self.get_logger().info('RTSP stream opened')
            return True
        except Exception as exc:
            self.get_logger().warning(f'RTSP open error: {exc}')
            return False

    def load_camera_info(self):
        url = self.get_parameter('camera_info_url').get_parameter_value().string_value
        if not url:
            self.get_logger().warning('No camera_info_url supplied – publishing default CameraInfo.')
            return
        try:
            file_path = url.replace('file://', '')
            with open(file_path, 'r') as fp:
                calib = yaml.safe_load(fp)
            self.camera_info = CameraInfo(
                height=calib['image_height'],
                width=calib['image_width'],
                distortion_model=calib['distortion_model'],
                d=calib['distortion_coefficients']['data'],
                k=calib['camera_matrix']['data'],
                r=calib['rectification_matrix']['data'],
                p=calib['projection_matrix']['data']
            )
            self.get_logger().info(f'Loaded calibration from {file_path}')
        except Exception as exc:
            self.get_logger().error(f'Could not load calibration: {exc}')

    def set_camera_info_cb(self, req, resp):
        self.camera_info = req.camera_info
        resp.success = True
        resp.status_message = 'CameraInfo updated'
        return resp

    def grab_loop(self):
        """Runs in a background thread – keeps last decoded frame."""
        while rclpy.ok():
            frame = self.capture_frame()
            if frame is not None:
                self.frame_buffer.append(frame)

    def get_latest_frame(self):
        return self.frame_buffer[-1] if self.frame_buffer else None

    # =========================================================
    # Image pipeline
    # =========================================================
    def capture_frame(self):
        with self.camera_lock:
            if not (self.cap and self.cap.isOpened()):
                if not self.initialize_camera():
                    return None
            # --- fast path ---
            _ = self.cap.grab()                     # grab newest buffer only
            ok, frame = self.cap.retrieve()         # decode one frame
            return frame if ok else None


    def publish_image(self):
        frame = self.get_latest_frame()
        if frame is None:
            return
        stamp = self.get_clock().now().to_msg()
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        img_msg.header.stamp = stamp
        img_msg.header.frame_id = self.frame_id
        self.image_pub.publish(img_msg)

        self.camera_info.header.stamp = stamp
        self.camera_info.header.frame_id = self.frame_id
        self.camera_info_pub.publish(self.camera_info)

    # =========================================================
    # Zoom control helpers
    # =========================================================

    def _send_zoom_cmd(self, cmd):
        url = f'http://{self.camera_hostname}/cgi-bin/ptzctrl.cgi?ptzcmd&{cmd}&5'
        stop_url = f'http://{self.camera_hostname}/cgi-bin/ptzctrl.cgi?ptzcmd&zoomstop&5'
        with self.zoom_lock:
            try:
                r1 = self.http.request('GET', url, headers=self.headers, timeout=2.5)
                time.sleep(0.15)
                r2 = self.http.request('GET', stop_url, headers=self.headers, timeout=2.5)
                return r1.status == 200 and r2.status == 200
            except Exception as exc:
                self.get_logger().warning(f'Zoom command error: {exc}')
                return False

    def zoom_in_step(self):
        if self.zoom_level >= self.max_zoom_level:
            return
        if self._send_zoom_cmd('zoomin'):
            self.zoom_level += 1

    def zoom_out_step(self):
        if self.zoom_level <= 0:
            return
        if self._send_zoom_cmd('zoomout'):
            self.zoom_level -= 1

    def zoom_out_all(self):
        self.get_logger().info('Zooming out fully')
        for _ in range(self.max_zoom_level):
            self.zoom_out_step()

    def zoom_in_all(self):
        self.get_logger().info('Zooming in fully')
        for _ in range(self.max_zoom_level):
            self.zoom_in_step()

    def set_zoom_level(self, level):
        level = max(0, min(level, self.max_zoom_level))
        if level == self.zoom_level:
            return
        steps = abs(level - self.zoom_level)
        step_fn = self.zoom_out_step if level < self.zoom_level else self.zoom_in_step
        for _ in range(steps):
            step_fn()

    # =========================================================
    # ROS 2 interfaces
    # =========================================================
    def command_callback(self, msg: String):
        table = {
            'zoom_in': self.zoom_in_step,
            'zoom_out': self.zoom_out_step,
            'zoom_in_all': self.zoom_in_all,
            'zoom_out_all': self.zoom_out_all,
            'ping': self.ping_camera,
        }
        fn = table.get(msg.data)
        if fn:
            fn()
            self.get_logger().info(f'Command executed: {msg.data}')
        else:
            self.get_logger().warning(f'Unknown command: {msg.data}')

    def level_callback(self, msg: Int16):
        self.set_zoom_level(msg.data)
        self.get_logger().info(f'Zoom level set to {msg.data}')

    def capture_service_cb(self, req, resp):
        frame = self.capture_frame()
        if frame is None:
            resp.success = False
            resp.message = 'Capture failed'
            return resp
        filename = os.path.join(
            self.save_dir,
            f'capture_{datetime.now():%Y%m%d_%H%M%S}_zoom{self.zoom_level}.jpg'
        )
        cv2.imwrite(filename, frame)
        resp.success = True
        resp.message = f'Saved {filename}'
        return resp

    def ping_camera(self):
        try:
            r = self.http.request('GET', f'http://{self.camera_hostname}', timeout=1.0)
            status = 'up' if r.status == 200 else 'down'
        except Exception:
            status = 'down'
        self.get_logger().info(f'Camera is {status}')


def main():
    rclpy.init()
    node = ZoomController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
