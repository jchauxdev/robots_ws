"""Servidor de accion didactico (Día 1, DA-03): cuenta regresiva con realimentacion y cancelacion."""
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from jc_interfaces.action import Countdown


class CountdownServer(Node):
    def __init__(self):
        super().__init__("countdown_server")
        ActionServer(self, Countdown, "/demo/countdown", self.execute,
                     cancel_callback=lambda _g: CancelResponse.ACCEPT)

    def execute(self, gh):
        fb = Countdown.Feedback()
        for r in range(gh.request.seconds, 0, -1):
            if gh.is_cancel_requested:
                gh.canceled()
                return Countdown.Result(finished=False)
            fb.remaining = r
            gh.publish_feedback(fb)
            time.sleep(1.0)
        gh.succeed()
        return Countdown.Result(finished=True)


def main():
    rclpy.init()
    ex = MultiThreadedExecutor()
    ex.add_node(CountdownServer())
    try:
        ex.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    rclpy.try_shutdown()
