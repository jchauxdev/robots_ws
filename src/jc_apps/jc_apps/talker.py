"""Publicador didactico (Día 1, DA-03): publica el estado de la celda en /celda/estado.

    ros2 run jc_apps talker --ros-args -p rate_hz:=2.0
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


class Talker(Node):
    def __init__(self):
        super().__init__("talker")
        self.declare_parameter("rate_hz", 1.0)
        self.pub = self.create_publisher(String, "/celda/estado", 10)
        self.n = 0
        self.timer = self.create_timer(1.0 / self.get_parameter("rate_hz").value, self.tick)

    def tick(self):
        self.n += 1
        msg = String(data=f"celda OK #{self.n}")
        self.pub.publish(msg)
        self.get_logger().info(f"Publicado: {msg.data}")


def main():
    rclpy.init()
    _spin(Talker())


def _spin(node):
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()
