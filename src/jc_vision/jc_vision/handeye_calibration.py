"""Calibracion mano-ojo (eye-in-hand) asistida - Día 3 (simulación, DA-09) y Día 4 (robot real, DA-16).

Procedimiento: fijar un marcador ArUco en la mesa; mover el robot a >= 10 poses
distintas (variando orientacion) con el marcador visible; en cada pose llamar
    ros2 service call /handeye_calibration/capture std_srvs/srv/Trigger
y al final
    ros2 service call /handeye_calibration/compute std_srvs/srv/Trigger
El resultado (T_brida_camara) se guarda en 'output_file' y se publica como TF estatica
con: ros2 launch jc_bringup real.launch.py handeye:=true
"""
import os

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from jc_vision import geometry_utils as gu
from jc_vision.vision_algorithms import (HANDEYE_METHODS, aruco_dictionary, detect_aruco,
                                         hand_eye_consistency, marker_pose, solve_hand_eye)


class HandEye(Node):
    def __init__(self):
        super().__init__("handeye_calibration")
        p = self.declare_parameter
        self.image_topic = p("image_topic", "/camera/image_raw").value
        self.info_topic = p("camera_info_topic", "/camera/camera_info").value
        self.base_frame = p("base_frame", "base_link").value
        self.gripper_frame = p("gripper_frame", "link6_flange").value
        self.child_frame = p("child_frame", "camera_calibrated_optical").value
        self.marker_id = int(p("marker_id", 7).value)
        self.length = float(p("marker_length", 0.030).value)
        self.dict = aruco_dictionary(p("dictionary", "DICT_4X4_50").value)
        self.method = p("method", "PARK").value
        self.output = os.path.expanduser(p("output_file", "~/robots_ws/handeye_result.yaml").value)

        self.bridge, self.K, self.D, self.last = CvBridge(), None, None, None
        self.A, self.B = [], []  # T_base_gripper, T_cam_target
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(CameraInfo, self.info_topic, self._info, qos_profile_sensor_data)
        self.create_subscription(Image, self.image_topic, self._img, qos_profile_sensor_data)
        self.create_service(Trigger, "~/capture", self._capture)
        self.create_service(Trigger, "~/compute", self._compute)
        self.create_service(Trigger, "~/reset", self._reset)
        self.get_logger().info("Listo. Servicios: ~/capture, ~/compute, ~/reset")

    def _info(self, m):
        self.K, self.D = gu.K_from_camera_info(m)

    def _img(self, m):
        self.last = m

    def _capture(self, _req, res):
        if self.last is None or self.K is None:
            res.success, res.message = False, "Sin imagen o CameraInfo"
            return res
        gray = cv2.cvtColor(self.bridge.imgmsg_to_cv2(self.last, "bgr8"), cv2.COLOR_BGR2GRAY)
        corners, ids = detect_aruco(gray, self.dict)
        if ids is None or self.marker_id not in ids.ravel():
            res.success, res.message = False, f"Marcador {self.marker_id} no visible"
            return res
        c = corners[list(ids.ravel()).index(self.marker_id)]
        rvec, tvec = marker_pose(c, self.length, self.K, self.D)
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, self.gripper_frame, Time(),
                                                 timeout=Duration(seconds=0.5))
        except Exception as e:
            res.success, res.message = False, f"TF: {e}"
            return res
        self.A.append(gu.transform_msg_to_T(tf.transform))
        self.B.append(gu.make_T(cv2.Rodrigues(rvec)[0], tvec.ravel()))
        res.success, res.message = True, f"Muestra {len(self.A)} capturada (z camara-marcador = {tvec[2,0]:.3f} m)"
        self.get_logger().info(res.message)
        return res

    def _compute(self, _req, res):
        if len(self.A) < 5:
            res.success, res.message = False, f"Se requieren >= 5 muestras (hay {len(self.A)}); recomendado >= 10"
            return res
        report, best = [], None
        for name in HANDEYE_METHODS:
            X = solve_hand_eye(self.A, self.B, name)
            _, mean_e, max_e = hand_eye_consistency(self.A, self.B, X)
            report.append(f"{name}: t={np.round(X[:3,3],4).tolist()} disp_media={mean_e*1000:.1f} mm max={max_e*1000:.1f} mm")
            if name == self.method:
                best = (X, mean_e, max_e)
        for line in report:
            self.get_logger().info(line)
        X, mean_e, max_e = best
        out = {"parent_frame": self.gripper_frame, "child_frame": self.child_frame,
               "translation": [float(v) for v in X[:3, 3]],
               "rotation_xyzw": [float(v) for v in gu.matrix_to_quat(X[:3, :3])],
               "method": self.method, "samples": len(self.A),
               "consistency_mean_mm": round(mean_e * 1000, 2), "consistency_max_mm": round(max_e * 1000, 2)}
        os.makedirs(os.path.dirname(self.output), exist_ok=True)
        with open(self.output, "w", encoding="utf-8") as f:
            yaml.safe_dump(out, f, sort_keys=False)
        res.success = True
        res.message = f"Guardado en {self.output}. Dispersion media {mean_e*1000:.1f} mm"
        return res

    def _reset(self, _req, res):
        self.A, self.B = [], []
        res.success, res.message = True, "Muestras borradas"
        return res


def main():
    rclpy.init()
    n = HandEye()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    n.destroy_node()
    rclpy.try_shutdown()
