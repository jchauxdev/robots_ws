"""Servidor de servicio didactico (Día 1, DA-03) con interfaz propia jc_interfaces/srv/SetGripper.

    ros2 run jc_apps gripper_service_demo
    ros2 service call /demo/set_gripper jc_interfaces/srv/SetGripper "{opening: 0.5}"
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from jc_interfaces.srv import SetGripper


class GripperDemo(Node):
    def __init__(self):
        super().__init__("gripper_service_demo")
        self.create_service(SetGripper, "/demo/set_gripper", self.cb)
        self.opening = 1.0

    def cb(self, req, res):
        if not 0.0 <= req.opening <= 1.0:
            res.success, res.message = False, "opening debe estar en [0, 1]"
        else:
            self.opening = req.opening
            res.success, res.message = True, f"Pinza al {req.opening*100:.0f} % (valor pymycobot {round(req.opening*100)})"
        self.get_logger().info(res.message)
        return res


def main():
    rclpy.init()
    _spin(GripperDemo())


def _spin(node):
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()
