"""Parametros (Día 1, DA-03): declaracion, lectura, validacion y cambio en caliente.

    ros2 run jc_apps param_demo
    ros2 param set /param_demo velocidad 0.5
    ros2 param set /param_demo velocidad 2.0     # rechazado por la validacion
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult, FloatingPointRange
from rclpy.node import Node


class ParamDemo(Node):
    def __init__(self):
        super().__init__("param_demo")
        self.declare_parameter(
            "velocidad", 0.3,
            ParameterDescriptor(description="Escalado de velocidad del cobot (0-1]",
                                floating_point_range=[FloatingPointRange(from_value=0.01, to_value=1.0)]))
        self.declare_parameter("modo", "simulacion")
        self.add_on_set_parameters_callback(self.on_set)
        self.create_timer(2.0, lambda: self.get_logger().info(
            f"velocidad={self.get_parameter('velocidad').value} modo={self.get_parameter('modo').value}"))

    def on_set(self, params):
        for p in params:
            if p.name == "modo" and p.value not in ("simulacion", "real"):
                return SetParametersResult(successful=False, reason="modo: simulacion | real")
        return SetParametersResult(successful=True)


def main():
    rclpy.init()
    _spin(ParamDemo())


def _spin(node):
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()
