"""jetcobot_bridge: puente ROS 2 <-> JetCobot (pymycobot) para MoveIt 2.

Expone las MISMAS interfaces que ros2_control usa en simulacion, de modo que
MoveIt 2 y los nodos de aplicacion funcionan igual en Gazebo y en el robot real:

  Publica  : /joint_states (sensor_msgs/JointState)  [rad, m]
  Acciones : /arm_controller/follow_joint_trajectory      (control_msgs/FollowJointTrajectory)
             /gripper_controller/follow_joint_trajectory  (control_msgs/FollowJointTrajectory)
  Servicios: ~/power_on, ~/power_off, ~/release_servos, ~/focus_servos, ~/stop (std_srvs/Trigger)

Parametros principales: port, baud, simulate, publish_rate, stream_rate,
max_joint_speed_deg_s, joint_offsets_deg, goal_tolerance_deg, finger_max_opening.

Notas de diseno:
  * El bus serie no admite accesos concurrentes: todas las llamadas a pymycobot
    se serializan con un Lock (igual que RobotController en las apps del instructor).
  * pymycobot puede devolver -1 en vez de una lista cuando el bus esta ocupado;
    se valida el tipo/longitud antes de usar la respuesta.
  * La trayectoria se re-muestrea a 'stream_rate' Hz y se envia con send_angles en
    modo refresco (set_fresh_mode(1)), que ejecuta siempre el ultimo comando.
"""
import math
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
FINGER_JOINTS = ["gripper_left_joint", "gripper_right_joint"]


def _dur(d):
    return d.sec + d.nanosec * 1e-9


class JetCobotBridge(Node):
    def __init__(self):
        super().__init__("jetcobot_bridge")
        p = self.declare_parameter
        self.port = p("port", "/dev/ttyUSB0").value
        self.baud = int(p("baud", 1000000).value)
        self.simulate = bool(p("simulate", False).value)
        self.publish_rate = float(p("publish_rate", 15.0).value)
        self.stream_rate = float(p("stream_rate", 10.0).value)
        self.max_deg_s = float(p("max_joint_speed_deg_s", 150.0).value)
        self.min_speed = int(p("min_speed", 10).value)
        self.max_speed = int(p("max_speed", 80).value)
        self.offsets = list(p("joint_offsets_deg", [0.0] * 6).value)
        self.tol_deg = float(p("goal_tolerance_deg", 2.0).value)
        self.settle_timeout = float(p("settle_timeout", 15.0).value)
        self.finger_max = float(p("finger_max_opening", 0.017).value)
        self.gripper_speed = int(p("gripper_speed", 50).value)

        self.lock = threading.Lock()
        self.mc = self._connect()
        self.q_deg = [0.0] * 6
        self.grip = 100
        self._tick = 0

        self.pub = self.create_publisher(JointState, "joint_states", 10)
        poll_group = MutuallyExclusiveCallbackGroup()
        self.create_timer(1.0 / self.publish_rate, self._poll, callback_group=poll_group)

        act_group = ReentrantCallbackGroup()
        self.arm_as = ActionServer(
            self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory",
            execute_callback=self._exec_arm, goal_callback=self._goal_arm,
            cancel_callback=lambda _g: CancelResponse.ACCEPT, callback_group=act_group)
        self.grip_as = ActionServer(
            self, FollowJointTrajectory, "/gripper_controller/follow_joint_trajectory",
            execute_callback=self._exec_gripper,
            cancel_callback=lambda _g: CancelResponse.ACCEPT, callback_group=act_group)

        srv_group = MutuallyExclusiveCallbackGroup()
        for name, fn in [("power_on", "power_on"), ("power_off", "power_off"),
                         ("release_servos", "release_all_servos"),
                         ("focus_servos", "focus_all_servos"), ("stop", "stop")]:
            self.create_service(Trigger, f"~/{name}", self._make_trigger(fn),
                                callback_group=srv_group)
        self.get_logger().info(
            f"JetCobot listo ({'SIMULADO' if self.simulate else self.port + ' @ ' + str(self.baud)})")

    # ------------------------------------------------------------------ conexion
    def _connect(self):
        if self.simulate:
            from jc_driver.fake_mycobot import FakeMyCobot280
            return FakeMyCobot280()
        from pymycobot import MyCobot280  # pymycobot >= 3.6
        mc = MyCobot280(self.port, self.baud)
        time.sleep(0.1)
        try:
            mc.power_on()
            if mc.get_fresh_mode() != 1:
                mc.set_fresh_mode(1)
        except Exception as e:  # el firmware antiguo puede no soportarlo
            self.get_logger().warn(f"No se pudo configurar el modo refresco: {e}")
        return mc

    def _call(self, fn, *args):
        with self.lock:
            return getattr(self.mc, fn)(*args)

    def _make_trigger(self, fn):
        def cb(_req, res):
            try:
                self._call(fn)
                res.success, res.message = True, fn
            except Exception as e:
                res.success, res.message = False, str(e)
            return res
        return cb

    # ------------------------------------------------------------------ estado
    def _poll(self):
        try:
            a = self._call("get_angles")
            if isinstance(a, (list, tuple)) and len(a) == 6:
                self.q_deg = [float(x) - o for x, o in zip(a, self.offsets)]
            self._tick += 1
            if self._tick % 5 == 0:
                g = self._call("get_gripper_value")
                if isinstance(g, (int, float)) and 0 <= g <= 100:
                    self.grip = g
        except Exception as e:
            self.get_logger().warn(f"Lectura serie fallida: {e}", throttle_duration_sec=5.0)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        f = self.finger_max * self.grip / 100.0
        msg.name = ARM_JOINTS + FINGER_JOINTS
        msg.position = [math.radians(d) for d in self.q_deg] + [f, f]
        self.pub.publish(msg)

    # ------------------------------------------------------------------ brazo
    def _goal_arm(self, goal):
        names = list(goal.trajectory.joint_names)
        if sorted(names) != sorted(ARM_JOINTS) or not goal.trajectory.points:
            self.get_logger().error(f"Trayectoria rechazada: articulaciones {names}")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _speed_pct(self, dq_deg, dt):
        v = max(abs(x) for x in dq_deg) / max(dt, 1e-3)
        pct = int(round(100.0 * v / self.max_deg_s))
        return max(self.min_speed, min(self.max_speed, pct))

    def _exec_arm(self, gh):
        traj = gh.request.trajectory
        idx = [list(traj.joint_names).index(j) for j in ARM_JOINTS]
        pts = [([math.degrees(pt.positions[i]) for i in idx], _dur(pt.time_from_start))
               for pt in traj.points]
        # re-muestreo: como maximo 'stream_rate' comandos por segundo (+ ultimo punto)
        period = 1.0 / self.stream_rate
        sel, last_t = [], -1e9
        for q, t in pts:
            if t - last_t >= period:
                sel.append((q, t))
                last_t = t
        if sel[-1] is not pts[-1]:
            sel.append(pts[-1])

        fb = FollowJointTrajectory.Feedback()
        fb.joint_names = ARM_JOINTS
        t0 = time.monotonic()
        prev_q, prev_t = list(self.q_deg), 0.0
        for q, t in sel:
            if gh.is_cancel_requested:
                self._call("stop")
                gh.canceled()
                return FollowJointTrajectory.Result()
            wait = t0 + t - time.monotonic()
            if wait > 0:
                time.sleep(wait)
            dq = [a - b for a, b in zip(q, prev_q)]
            cmd = [a + o for a, o in zip(q, self.offsets)]
            self._call("send_angles", [round(x, 2) for x in cmd], self._speed_pct(dq, t - prev_t))
            prev_q, prev_t = q, t
            fb.actual.positions = [math.radians(d) for d in self.q_deg]
            fb.desired.positions = [math.radians(d) for d in q]
            gh.publish_feedback(fb)

        # esperar a que el robot alcance el ultimo punto
        target = sel[-1][0]
        deadline = time.monotonic() + self.settle_timeout
        res = FollowJointTrajectory.Result()
        while time.monotonic() < deadline:
            err = max(abs(a - b) for a, b in zip(self.q_deg, target))
            if err <= self.tol_deg:
                res.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                gh.succeed()
                return res
            if gh.is_cancel_requested:
                self._call("stop")
                gh.canceled()
                return res
            time.sleep(0.05)
        err = max(abs(a - b) for a, b in zip(self.q_deg, target))
        res.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
        res.error_string = f"Error final {err:.1f} deg > {self.tol_deg} deg"
        self.get_logger().warn(res.error_string)
        gh.abort()
        return res

    # ------------------------------------------------------------------ pinza
    def _exec_gripper(self, gh):
        traj = gh.request.trajectory
        res = FollowJointTrajectory.Result()
        if not traj.points:
            gh.abort()
            return res
        last = traj.points[-1]
        pos = max(last.positions) if last.positions else 0.0
        value = int(round(100.0 * max(0.0, min(self.finger_max, pos)) / self.finger_max))
        self._call("set_gripper_value", value, self.gripper_speed)
        time.sleep(0.3)
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            try:
                if not self._call("is_gripper_moving"):
                    break
            except Exception:
                break
            time.sleep(0.1)
        self.grip = value
        res.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        gh.succeed()
        return res

    def destroy_node(self):
        try:
            self._call("close")
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = JetCobotBridge()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
