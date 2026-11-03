"""Practica Día 2 (DA-05 y DA-07) (Python + MoveIt via servicios): dibuja un cuadrado con el TCP.

    ros2 run jc_apps square_demo --ros-args -p side:=0.04 -p z:=0.14
"""
import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from jc_apps.motion_client import MotionClient


def main():
    rclpy.init()
    node = Node("square_demo")
    side = float(node.declare_parameter("side", 0.04).value)
    x0 = float(node.declare_parameter("x", 0.17).value)
    y0 = float(node.declare_parameter("y", -0.08).value)
    z = float(node.declare_parameter("z", 0.14).value)
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()
    mc = MotionClient(node)
    if not mc.wait(30.0):
        node.get_logger().error("motion_server no disponible")
        ex.shutdown()
        th.join(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()
        return
    log = node.get_logger().info
    log(f"ready: {mc.named('ready')}")
    log(f"inicio: {mc.pose(x0, y0, z)}")
    for dx, dy in [(side, 0), (side, side), (0, side), (0, 0)]:
        log(f"lado -> ({x0+dx:.3f},{y0+dy:.3f}): {mc.pose(x0 + dx, y0 + dy, z, cartesian=True)}")
    log(f"ready: {mc.named('ready')}")
    ex.shutdown()
    th.join(timeout=2.0)
    node.destroy_node()
    rclpy.shutdown()
