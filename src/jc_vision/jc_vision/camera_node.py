"""camera_node: publica una camara USB (V4L2) como sensor_msgs/Image + CameraInfo.

    ros2 run jc_vision camera_node --ros-args -p device:=/dev/video0 -p camera_info_file:=<yaml>
Alternativas mantenidas por la comunidad: paquetes usb_cam y v4l2_camera.
"""
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from jc_vision.geometry_utils import load_camera_yaml


class CameraNode(Node):
    def __init__(self):
        super().__init__("camera_node")
        p = self.declare_parameter
        dev = p("device", "/dev/video0").value
        self.w, self.h = int(p("width", 640).value), int(p("height", 480).value)
        fps = float(p("fps", 30.0).value)
        self.frame_id = p("frame_id", "camera_optical_frame").value
        info_file = p("camera_info_file", "").value

        src = int(dev) if str(dev).isdigit() else dev
        self.cap = cv2.VideoCapture(src, cv2.CAP_V4L2) if isinstance(src, str) else cv2.VideoCapture(src)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.w)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.h)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        if not self.cap.isOpened():
            raise RuntimeError(f"No se pudo abrir {dev}")

        self.info = CameraInfo()
        self.info.width, self.info.height = self.w, self.h
        if info_file:
            d, K, D = load_camera_yaml(info_file)
            self.info.k = K.ravel().tolist()
            self.info.d = D.ravel().tolist()
            self.info.distortion_model = d.get("distortion_model", "plumb_bob")
            self.info.r = np.array(d.get("rectification_matrix", {"data": np.eye(3).ravel().tolist()})["data"]).tolist()
            P = d.get("projection_matrix", {}).get("data")
            self.info.p = P if P else [K[0, 0], 0., K[0, 2], 0., 0., K[1, 1], K[1, 2], 0., 0., 0., 1., 0.]
        else:
            self.get_logger().warn("Sin calibracion: se usa un modelo nominal (FOV 60 grados)")
            f = (self.w / 2) / np.tan(np.radians(30))
            self.info.k = [f, 0., self.w / 2, 0., f, self.h / 2, 0., 0., 1.]
            self.info.d = [0.] * 5
            self.info.distortion_model = "plumb_bob"
            self.info.r = np.eye(3).ravel().tolist()
            self.info.p = [f, 0., self.w / 2, 0., 0., f, self.h / 2, 0., 0., 0., 1., 0.]

        self.bridge = CvBridge()
        self.pub_img = self.create_publisher(Image, "/camera/image_raw", qos_profile_sensor_data)
        self.pub_info = self.create_publisher(CameraInfo, "/camera/camera_info", qos_profile_sensor_data)
        self.create_timer(1.0 / fps, self._grab)

    def _grab(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn("Fallo de captura", throttle_duration_sec=2.0)
            return
        stamp = self.get_clock().now().to_msg()
        m = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        m.header.stamp, m.header.frame_id = stamp, self.frame_id
        self.info.header.stamp, self.info.header.frame_id = stamp, self.frame_id
        self.pub_img.publish(m)
        self.pub_info.publish(self.info)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main():
    rclpy.init()
    n = CameraNode()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    n.destroy_node()
    rclpy.try_shutdown()
