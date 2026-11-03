"""Mueve el robot por N poses y captura muestras para handeye_calibration (prueba/automatizacion)."""
import math
import time
import subprocess
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_srvs.srv import Trigger

POSES = [[0, 10, -60, -40, 0, 0], [12, 10, -60, -42, 15, 20], [-12, 12, -62, -36, -15, -20],
         [8, 4, -52, -52, 18, 35], [-8, 16, -68, -28, -18, -35], [0, 4, -50, -54, 0, 50],
         [15, 12, -60, -34, -12, 12], [-15, 6, -58, -46, 12, -12], [4, 16, -70, -24, 18, 0],
         [-4, 2, -48, -54, -18, 0], [10, 8, -56, -50, 8, -40], [-10, 12, -62, -32, -8, 40],
         [0, 14, -66, -30, 0, 70], [0, 4, -52, -50, 0, -70]]


def main():
    rclpy.init()
    n = Node("handeye_auto")
    n.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
    ac = ActionClient(n, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
    cap = n.create_client(Trigger, "/handeye_calibration/capture")
    comp = n.create_client(Trigger, "/handeye_calibration/compute")
    ac.wait_for_server(); cap.wait_for_service()
    for q in POSES:
        g = FollowJointTrajectory.Goal()
        g.trajectory.joint_names = [f"joint{i}" for i in range(1, 7)]
        g.trajectory.points = [JointTrajectoryPoint(positions=[math.radians(a) for a in q],
                                                    time_from_start=Duration(sec=2))]
        f = ac.send_goal_async(g); rclpy.spin_until_future_complete(n, f)
        r = f.result().get_result_async(); rclpy.spin_until_future_complete(n, r)
        time.sleep(3.0)
        f = cap.call_async(Trigger.Request()); rclpy.spin_until_future_complete(n, f)
        print(q, f.result().message)
    f = comp.call_async(Trigger.Request()); rclpy.spin_until_future_complete(n, f)
    print("COMPUTE:", f.result().message)


if __name__ == "__main__":
    main()
