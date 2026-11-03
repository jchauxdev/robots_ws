"""Cliente FollowJointTrajectory (Día 2, practica Python: DA-07; robot real en DA-15): trayectoria articular por puntos.

Funciona igual en Gazebo (ros2_control) y en el robot real (jc_driver), porque ambos
exponen /arm_controller/follow_joint_trajectory.
    ros2 run jc_apps trajectory_demo
    ros2 run jc_apps trajectory_demo --ros-args -p segment_time:=3.0
"""
import math

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint

JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
# Puntos de paso en grados (ready -> barrido -> ready)
WAYPOINTS_DEG = [
    [0, 0, -90, 0, 0, 0],
    [30, 10, -100, 0, 0, 0],
    [-30, 10, -100, 0, 0, 0],
    [0, 10, -60, -40, 0, 0],
    [0, 0, -90, 0, 0, 0],
]


class TrajectoryDemo(Node):
    def __init__(self):
        super().__init__("trajectory_demo")
        self.seg = float(self.declare_parameter("segment_time", 2.5).value)
        self.cli = ActionClient(self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")

    def run(self):
        self.get_logger().info("Esperando el servidor de accion...")
        self.cli.wait_for_server()
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        for k, wp in enumerate(WAYPOINTS_DEG, start=1):
            pt = JointTrajectoryPoint()
            pt.positions = [math.radians(a) for a in wp]
            t = k * self.seg
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        fut = self.cli.send_goal_async(goal, feedback_callback=self.on_fb)
        rclpy.spin_until_future_complete(self, fut)
        gh = fut.result()
        if not gh.accepted:
            self.get_logger().error("Meta rechazada")
            return False
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res)
        code = res.result().result.error_code
        self.get_logger().info(f"Resultado: error_code={code} (0 = SUCCESSFUL)")
        return code == 0

    def on_fb(self, fb):
        a = fb.feedback.actual.positions
        if a:
            self.get_logger().info("actual [deg]: " + ", ".join(f"{math.degrees(x):6.1f}" for x in a),
                                   throttle_duration_sec=1.0)


def main():
    rclpy.init()
    ok = TrajectoryDemo().run()
    rclpy.shutdown()
    raise SystemExit(0 if ok else 1)
