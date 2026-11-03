"""joint_state_follower: el robot real 'sigue' al modelo de RViz (Día 3, Parte B: DA-14).

Equivalente didactico a 'jetcobot_driver sync_plan' (Yahboom) y 'sync_plan.py'
(Elephant Robotics): se suscribe a un topico JointState (por defecto
/joint_states publicado por joint_state_publisher_gui) y envia send_angles.
NO ejecutar junto con jetcobot_bridge (ambos usan el puerto serie).
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]


class Follower(Node):
    def __init__(self):
        super().__init__("joint_state_follower")
        self.port = self.declare_parameter("port", "/dev/ttyUSB0").value
        self.baud = int(self.declare_parameter("baud", 1000000).value)
        self.speed = int(self.declare_parameter("speed", 30).value)
        self.min_delta = float(self.declare_parameter("min_delta_deg", 0.5).value)
        sim = bool(self.declare_parameter("simulate", False).value)
        if sim:
            from jc_driver.fake_mycobot import FakeMyCobot280 as Robot
        else:
            from pymycobot import MyCobot280 as Robot
        self.mc = Robot(self.port, self.baud)
        self.last = None
        self.create_subscription(JointState, "joint_states", self.cb, 10)
        self.get_logger().info("Siguiendo /joint_states -> send_angles (Ctrl+C para salir)")

    def cb(self, msg):
        d = dict(zip(msg.name, msg.position))
        if not all(j in d for j in ARM_JOINTS):
            return
        q = [round(math.degrees(d[j]), 2) for j in ARM_JOINTS]
        if self.last and max(abs(a - b) for a, b in zip(q, self.last)) < self.min_delta:
            return
        self.mc.send_angles(q, self.speed)
        self.last = q


def main():
    rclpy.init()
    n = Follower()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    rclpy.try_shutdown()
