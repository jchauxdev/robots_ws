"""Compara la FK numerica (kinematics.py) con la TF de robot_state_publisher - Día 2 (DA-05).

Con la celda corriendo (mock, Gazebo o real):
    ros2 run jc_apps fk_check
Imprime q [deg], la posicion del TCP por FK propia y por TF, y la diferencia en mm.
"""
import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from jc_apps.kinematics import SerialChain


class FkCheck(Node):
    def __init__(self):
        super().__init__("fk_check")
        self.chain, self.q = None, None
        self.tf = Buffer()
        TransformListener(self.tf, self)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/robot_description", self.on_urdf, qos)
        self.create_subscription(JointState, "/joint_states", self.on_js, 10)
        self.create_timer(1.0, self.report)

    def on_urdf(self, msg):
        self.chain = SerialChain(msg.data, "world", "tcp")
        self.get_logger().info(f"Cadena: {self.chain.names}")

    def on_js(self, msg):
        d = dict(zip(msg.name, msg.position))
        if self.chain and all(n in d for n in self.chain.names):
            self.q = np.array([d[n] for n in self.chain.names])

    def report(self):
        if self.chain is None or self.q is None:
            return
        T = self.chain.fk(self.q)
        try:
            t = self.tf.lookup_transform("world", "tcp", Time()).transform.translation
        except Exception as e:
            self.get_logger().warn(str(e))
            return
        p_tf = np.array([t.x, t.y, t.z])
        self.get_logger().info(
            f"q={np.round(np.degrees(self.q),1).tolist()} FK={np.round(T[:3,3]*1000,1).tolist()} mm "
            f"TF={np.round(p_tf*1000,1).tolist()} mm |dif|={np.linalg.norm(T[:3,3]-p_tf)*1000:.3f} mm "
            f"w={self.chain.manipulability(self.q):.2e}")


def main():
    rclpy.init()
    _spin(FkCheck())


def _spin(node):
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()
