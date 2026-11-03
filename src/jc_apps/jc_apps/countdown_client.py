"""Cliente de accion didactico (Día 1, DA-03)."""
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from jc_interfaces.action import Countdown


def main():
    rclpy.init()
    node = Node("countdown_client")
    cli = ActionClient(node, Countdown, "/demo/countdown")
    cli.wait_for_server()
    goal = Countdown.Goal(seconds=int(sys.argv[1]) if len(sys.argv) > 1 else 5)
    fut = cli.send_goal_async(goal, feedback_callback=lambda f: node.get_logger().info(
        f"faltan {f.feedback.remaining} s"))
    rclpy.spin_until_future_complete(node, fut)
    res = fut.result().get_result_async()
    rclpy.spin_until_future_complete(node, res)
    node.get_logger().info(f"Resultado: finished={res.result().result.finished}")
    rclpy.shutdown()
