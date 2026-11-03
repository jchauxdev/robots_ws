"""Suscriptor didactico (Día 1, DA-03): escucha /celda/estado."""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


class Listener(Node):
    def __init__(self):
        super().__init__("listener")
        self.create_subscription(String, "/celda/estado", self.cb, 10)

    def cb(self, msg):
        self.get_logger().info(f"Recibido: {msg.data}")


def main():
    rclpy.init()
    _spin(Listener())


def _spin(node):
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()
