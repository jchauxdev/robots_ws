"""Camara sintetica: proyecta la escena conocida de la celda con la pose real de la camara (TF).

Sirve para validar la cadena pixel -> 3D sin GPU (el render de Gazebo requiere OpenGL).
Publica /wrist_camera/image_raw y /wrist_camera/camera_info (marco camera_optical_frame).
"""
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

from jc_vision import geometry_utils as gu

W, H, F = 640, 480, 554.2563
K = np.array([[F, 0, W / 2], [0, F, H / 2], [0, 0, 1.0]])
# (nombre, centro xyz de la cara superior, lado, color BGR, forma)
OBJ = [("rojo", (0.16, -0.06, 0.025), 0.025, (20, 20, 215), "sq"),
       ("verde", (0.11, -0.15, 0.025), 0.025, (30, 165, 20), "sq"),
       ("azul", (0.19, -0.14, 0.030), 0.025, (215, 50, 15), "circ")]
ARUCO = ((0.10, -0.04, 0.0006), 0.04)


class SynthCam(Node):
    def __init__(self):
        super().__init__("synthetic_camera")
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.br = CvBridge()
        tex = cv2.imread(self.declare_parameter("aruco_png", "").value, cv2.IMREAD_GRAYSCALE)
        self.tex = cv2.cvtColor(tex, cv2.COLOR_GRAY2BGR) if tex is not None else None
        self.pi = self.create_publisher(Image, "/wrist_camera/image_raw", 5)
        self.pc = self.create_publisher(CameraInfo, "/wrist_camera/camera_info", 5)
        self.create_timer(0.1, self.tick)

    def proj(self, T_cam_w, pts):
        P = (T_cam_w @ np.c_[pts, np.ones(len(pts))].T)[:3]
        return (K @ P)[:2] / P[2]

    def tick(self):
        try:
            T_w_c = gu.transform_msg_to_T(self.tf.lookup_transform("world", "camera_optical_frame", Time()).transform)
        except Exception:
            return
        T = gu.invert_T(T_w_c)
        img = np.full((H, W, 3), (190, 205, 215), np.uint8)
        if self.tex is not None:
            (x, y, z), s = ARUCO
            c = np.array([[x - s/2, y + s/2, z], [x + s/2, y + s/2, z], [x + s/2, y - s/2, z], [x - s/2, y - s/2, z]])
            dst = self.proj(T, c).T.astype(np.float32)
            h, w = self.tex.shape[:2]
            M = cv2.getPerspectiveTransform(np.float32([[0, 0], [w, 0], [w, h], [0, h]]), dst)
            warped = cv2.warpPerspective(self.tex, M, (W, H), borderValue=(0, 0, 0))
            mask = cv2.warpPerspective(np.full((h, w), 255, np.uint8), M, (W, H))
            img[mask > 0] = warped[mask > 0]
        for _, (x, y, z), s, col, kind in OBJ:
            if kind == "sq":
                c = np.array([[x - s/2, y - s/2, z], [x + s/2, y - s/2, z], [x + s/2, y + s/2, z], [x - s/2, y + s/2, z]])
            else:
                a = np.linspace(0, 2 * np.pi, 40, endpoint=False)
                c = np.c_[x + s/2 * np.cos(a), y + s/2 * np.sin(a), np.full_like(a, z)]
            cv2.fillPoly(img, [self.proj(T, c).T.round().astype(np.int32)], col)
        img = cv2.GaussianBlur(img, (3, 3), 0)
        stamp = self.get_clock().now().to_msg()
        m = self.br.cv2_to_imgmsg(img, "bgr8")
        m.header.stamp, m.header.frame_id = stamp, "camera_optical_frame"
        info = CameraInfo(width=W, height=H, distortion_model="plumb_bob", d=[0.0] * 5,
                          k=K.ravel().tolist(), r=np.eye(3).ravel().tolist(),
                          p=[F, 0., W / 2, 0., 0., F, H / 2, 0., 0., 0., 1., 0.])
        info.header = m.header
        self.pi.publish(m)
        self.pc.publish(info)


def main():
    rclpy.init()
    rclpy.spin(SynthCam())


if __name__ == "__main__":
    main()
