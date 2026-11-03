"""Clase base para detectores ROS 2: suscripcion imagen + CameraInfo, TF y publicacion.

Cada detector concreto implementa detect(bgr) -> lista de dicts con claves:
  label, shape, score, u, v, w, h, angle_rad, (opcional) T_cam_obj (4x4, pose en la camara)
La base calcula la pose 3D en 'base_frame':
  * si el detector entrega T_cam_obj (p. ej. ArUco): la transforma con TF;
  * si no: interseca el rayo del pixel con el plano z = plane_z (mesa + altura de la pieza).
"""
import math

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from jc_interfaces.msg import DetectedObject, DetectedObjectArray
from jc_vision import geometry_utils as gu


class DetectorBase(Node):
    def __init__(self, name):
        super().__init__(name)
        p = self.declare_parameter
        self.image_topic = p("image_topic", "/camera/image_raw").value
        self.info_topic = p("camera_info_topic", "/camera/camera_info").value
        self.base_frame = p("base_frame", "world").value
        self.camera_frame = p("camera_frame", "").value  # vacio => usar header.frame_id
        self.plane_z = float(p("plane_z", 0.025).value)
        self.publish_debug = bool(p("publish_debug", True).value)
        self.use_latest_tf = bool(p("use_latest_tf", True).value)

        self.bridge = CvBridge()
        self.K = self.D = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(CameraInfo, self.info_topic, self._on_info, qos_profile_sensor_data)
        self.create_subscription(Image, self.image_topic, self._on_image, qos_profile_sensor_data)
        self.pub = self.create_publisher(DetectedObjectArray, "/jc/detections", 10)
        self.pub_dbg = self.create_publisher(Image, "/jc/debug_image", 5)
        self.pub_mk = self.create_publisher(MarkerArray, "/jc/markers", 5)
        self.get_logger().info(f"{name}: imagen={self.image_topic} base={self.base_frame}")

    # -- a implementar por las subclases
    def detect(self, bgr):
        raise NotImplementedError

    def draw(self, bgr, dets):
        return bgr

    # --
    def _on_info(self, msg):
        self.K, self.D = gu.K_from_camera_info(msg)

    def _lookup(self, cam_frame, stamp):
        t = Time() if self.use_latest_tf else Time.from_msg(stamp)
        tf = self.tf_buffer.lookup_transform(self.base_frame, cam_frame, t, timeout=Duration(seconds=0.1))
        return gu.transform_msg_to_T(tf.transform)

    def _on_image(self, msg):
        if self.K is None:
            self.get_logger().warn("Esperando CameraInfo...", throttle_duration_sec=5.0)
            return
        bgr = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        dets = self.detect(bgr)
        cam_frame = self.camera_frame or msg.header.frame_id
        try:
            T_base_cam = self._lookup(cam_frame, msg.header.stamp)
        except TransformException as e:
            T_base_cam = None
            self.get_logger().warn(f"TF {self.base_frame}<-{cam_frame} no disponible: {e}",
                                   throttle_duration_sec=5.0)

        out = DetectedObjectArray()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.base_frame
        markers = MarkerArray()
        markers.markers.append(Marker(action=Marker.DELETEALL))
        for i, d in enumerate(dets):
            o = DetectedObject()
            o.header = out.header
            o.label, o.shape = d.get("label", ""), d.get("shape", "")
            o.score = float(d.get("score", 1.0))
            o.u, o.v = int(d["u"]), int(d["v"])
            o.width_px, o.height_px = float(d.get("w", 0)), float(d.get("h", 0))
            o.angle_rad = float(d.get("angle_rad", 0.0))
            if T_base_cam is not None:
                pose = self._pose_in_base(d, T_base_cam)
                if pose is not None:
                    o.pose, o.pose_valid = pose, True
                    markers.markers.append(self._marker(i, o))
            out.objects.append(o)
        self.pub.publish(out)
        self.pub_mk.publish(markers)
        if self.publish_debug and self.pub_dbg.get_subscription_count() > 0:
            dbg = self.draw(bgr.copy(), dets)
            m = self.bridge.cv2_to_imgmsg(dbg, "bgr8")
            m.header = msg.header
            self.pub_dbg.publish(m)

    def _pose_in_base(self, d, T_base_cam):
        p = Pose()
        if "T_cam_obj" in d:
            T = T_base_cam @ d["T_cam_obj"]
            q = gu.matrix_to_quat(T[:3, :3])
            xyz = T[:3, 3]
        else:
            xyz = gu.pixel_to_base(d["u"], d["v"], self.K, self.D, T_base_cam, self.plane_z)
            if xyz is None:
                return None
            yaw = gu.yaw_in_base(d["u"], d["v"], d.get("angle_rad", 0.0), self.K, self.D,
                                 T_base_cam, self.plane_z)
            q = gu.quat_from_rpy(0.0, 0.0, yaw)
        p.position.x, p.position.y, p.position.z = map(float, xyz)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q)
        return p

    def _marker(self, i, o):
        m = Marker()
        m.header = o.header
        m.ns, m.id = "detecciones", i
        m.type = Marker.CUBE
        m.pose = o.pose
        m.scale.x = m.scale.y = m.scale.z = 0.025
        m.color.a = 0.8
        m.color.r, m.color.g, m.color.b = {"rojo": (1., 0., 0.), "verde": (0., 1., 0.),
                                           "azul": (0., 0., 1.)}.get(o.label, (1., 1., 0.))
        m.lifetime = Duration(seconds=0.5).to_msg()
        return m


def spin(node_cls):
    rclpy.init()
    n = node_cls()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    n.destroy_node()
    rclpy.try_shutdown()
