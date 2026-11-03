"""Detector de marcadores ArUco con pose 6D (solvePnP IPPE_SQUARE) y difusion TF - Día 3 (simulación, DA-10) y Día 4 (robot real, DA-16)."""
import cv2
import numpy as np
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from jc_vision import geometry_utils as gu
from jc_vision.detector_base import DetectorBase, spin
from jc_vision.vision_algorithms import aruco_dictionary, detect_aruco, marker_pose


class ArucoDetector(DetectorBase):
    def __init__(self):
        super().__init__("aruco_detector")
        self.dict_name = self.declare_parameter("dictionary", "DICT_4X4_50").value
        self.length = float(self.declare_parameter("marker_length", 0.030).value)
        self.broadcast_tf = bool(self.declare_parameter("broadcast_tf", True).value)
        self.dictionary = aruco_dictionary(self.dict_name)
        self.tfb = TransformBroadcaster(self)
        self._last = []

    def detect(self, bgr):
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        corners, ids = detect_aruco(gray, self.dictionary)
        dets = []
        if ids is None:
            self._last = []
            return dets
        for c, i in zip(corners, ids.ravel()):
            rvec, tvec = marker_pose(c, self.length, self.K, self.D)
            if rvec is None:
                continue
            R, _ = cv2.Rodrigues(rvec)
            pts = c.reshape(4, 2)
            u, v = pts.mean(axis=0)
            edge = pts[1] - pts[0]
            dets.append({"label": f"aruco_{int(i)}", "shape": "marcador", "score": 1.0,
                         "u": u, "v": v, "w": float(np.linalg.norm(edge)), "h": float(np.linalg.norm(pts[3] - pts[0])),
                         "angle_rad": float(np.arctan2(edge[1], edge[0])),
                         "T_cam_obj": gu.make_T(R, tvec.ravel()), "rvec": rvec, "tvec": tvec, "corners": c})
        self._last = dets
        return dets

    def _on_image(self, msg):
        super()._on_image(msg)
        if not self.broadcast_tf:
            return
        cam = self.camera_frame or msg.header.frame_id
        for d in self._last:
            T = d["T_cam_obj"]
            q = gu.matrix_to_quat(T[:3, :3])
            t = TransformStamped()
            t.header.stamp, t.header.frame_id = msg.header.stamp, cam
            t.child_frame_id = d["label"]
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, T[:3, 3])
            (t.transform.rotation.x, t.transform.rotation.y,
             t.transform.rotation.z, t.transform.rotation.w) = map(float, q)
            self.tfb.sendTransform(t)

    def draw(self, img, dets):
        if dets:
            cv2.aruco.drawDetectedMarkers(img, [d["corners"] for d in dets],
                                          np.array([[int(d["label"].split("_")[1])] for d in dets]))
            for d in dets:
                cv2.drawFrameAxes(img, self.K, self.D, d["rvec"], d["tvec"], self.length * 0.7)
        return img


def main():
    spin(ArucoDetector)
