"""pick_place_node: aplicacion integrada vision + cobot (Día 3 en simulación, DA-12; Día 4 con el robot real, DA-17).

Accion:   /jc/pick_place  (jc_interfaces/action/PickPlace)
Entrada:  /jc/detections  (jc_interfaces/msg/DetectedObjectArray, poses en base_frame)
Salida:   llamadas a /jc/move_to_named, /jc/move_to_pose, /jc/set_gripper (jc_motion)

Ciclo de recogida:
  observe -> esperar detecciones estables -> abrir pinza -> aproximacion sobre la pieza
  -> descenso cartesiano -> cerrar -> ascenso cartesiano -> contenedor -> abrir -> observe
Modo automatico (auto_sort:=true): clasifica continuamente todas las piezas visibles.
Seguridad: dry_run:=true solo calcula y registra las poses sin mover el robot.
"""
import math
import statistics
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from jc_apps.motion_client import MotionClient
from jc_interfaces.action import PickPlace
from jc_interfaces.msg import DetectedObjectArray


def yaw_from_quat(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def best_tcp_yaw(obj_yaw, symmetry=math.pi / 2, nominal=-math.pi / 2):
    """Elige el yaw del TCP equivalente por simetria de la pieza mas cercano al nominal."""
    best, dmin = obj_yaw, 1e9
    for k in range(-4, 5):
        c = obj_yaw + k * symmetry
        d = abs(math.atan2(math.sin(c - nominal), math.cos(c - nominal)))
        if d < dmin:
            best, dmin = c, d
    return math.atan2(math.sin(best), math.cos(best))


class PickPlaceNode(Node):
    def __init__(self):
        super().__init__("pick_place_node", automatically_declare_parameters_from_overrides=True)
        g = self._p
        self.base_frame = g("base_frame", "world")
        self.approach = float(g("approach_height", 0.06))
        self.grasp_h = float(g("grasp_height", 0.012))
        self.lift = float(g("lift_height", 0.10))
        self.vel = float(g("velocity_scaling", 0.3))
        self.observe = g("observe_state", "observe")
        self.settle = float(g("settle_time", 1.0))
        self.window = float(g("detection_window", 1.0))
        self.min_det = int(g("min_detections", 3))
        self.det_timeout = float(g("detection_timeout", 5.0))
        self.dry = bool(g("dry_run", False))
        self.auto = bool(g("auto_sort", False))
        self.bins = {name: list(p.value) for name, p in self.get_parameters_by_prefix("bins").items()}
        if "default" not in self.bins:
            self.bins["default"] = [0.02, 0.17, 0.05]

        self.cb = ReentrantCallbackGroup()
        self.buffer, self.lock = [], threading.Lock()
        self.create_subscription(DetectedObjectArray, "/jc/detections", self.on_det, 10,
                                 callback_group=self.cb)
        self.mc = MotionClient(self)
        ActionServer(self, PickPlace, "/jc/pick_place", self.execute, callback_group=self.cb,
                     cancel_callback=lambda _g: CancelResponse.ACCEPT)
        self.get_logger().info(f"pick_place_node listo. Contenedores: {self.bins} dry_run={self.dry}")
        if self.auto:
            threading.Thread(target=self.auto_loop, daemon=True).start()

    def _p(self, name, default):
        if not self.has_parameter(name):
            self.declare_parameter(name, default)
        return self.get_parameter(name).value

    # ---------------------------------------------------------- percepcion
    def on_det(self, msg):
        now = time.monotonic()
        with self.lock:
            self.buffer.append((now, msg))
            self.buffer = [(t, m) for t, m in self.buffer if now - t < 5.0]

    def locate(self, label):
        """Mediana de las detecciones de 'label' (robusto a ruido).

        Acumula detecciones recibidas DESPUES de llamar (robot ya detenido) durante al menos
        'detection_window' s y hasta reunir 'min_detections' muestras o agotar 'detection_timeout'.
        """
        t0 = time.monotonic()
        while True:
            time.sleep(0.1)
            now = time.monotonic()
            with self.lock:
                msgs = [m for t, m in self.buffer if t >= t0]
            xs, ys, zs, yaws, lab = [], [], [], [], None
            for m in msgs:
                cands = [o for o in m.objects if o.pose_valid and (not label or o.label == label)]
                if not cands:
                    continue
                o = min(cands, key=lambda o: math.hypot(o.pose.position.x, o.pose.position.y))
                lab = lab or o.label
                if o.label != lab:
                    continue
                xs.append(o.pose.position.x)
                ys.append(o.pose.position.y)
                zs.append(o.pose.position.z)
                yaws.append(yaw_from_quat(o.pose.orientation))
            enough = len(xs) >= self.min_det and now - t0 >= self.window
            if enough or now - t0 > self.det_timeout:
                break
        if len(xs) < self.min_det:
            return None
        return lab, statistics.median(xs), statistics.median(ys), statistics.median(zs), statistics.median(yaws)

    # ---------------------------------------------------------- movimiento
    def step(self, gh, stage, progress, fn, *args, **kw):
        if gh is not None:
            fb = PickPlace.Feedback(stage=stage, progress=float(progress))
            gh.publish_feedback(fb)
        self.get_logger().info(f"[{progress*100:3.0f} %] {stage}")
        if self.dry:
            return True
        ok, msg = fn(*args, **kw)
        if not ok:
            raise RuntimeError(f"{stage}: {msg}")
        return True

    def pick_and_place(self, label, place=None, gh=None):
        mc, v = self.mc, self.vel
        self.step(gh, "observar", 0.05, mc.named, self.observe, v)
        time.sleep(self.settle)
        loc = self.locate(label)
        if loc is None:
            return False, f"No se detecto '{label or 'ninguna pieza'}' de forma estable"
        lab, x, y, z, yaw_obj = loc
        yaw = best_tcp_yaw(yaw_obj)
        zg = max(self.grasp_h, z - 0.013)
        self.get_logger().info(f"Pieza {lab}: ({x:.3f}, {y:.3f}, {z:.3f}) yaw_obj={math.degrees(yaw_obj):.1f} "
                               f"-> TCP yaw={math.degrees(yaw):.1f} z_agarre={zg:.3f}")
        bx, by, bz = place if place else self.bins.get(lab, self.bins["default"])
        self.step(gh, "abrir pinza", 0.15, mc.gripper, 1.0)
        self.step(gh, "aproximacion", 0.30, mc.pose, x, y, zg + self.approach, yaw, self.base_frame, False, v)
        self.step(gh, "descenso", 0.45, mc.pose, x, y, zg, yaw, self.base_frame, True, v * 0.5)
        self.step(gh, "cerrar pinza", 0.55, mc.gripper, 0.0)
        self.step(gh, "ascenso", 0.65, mc.pose, x, y, zg + self.lift, yaw, self.base_frame, True, v * 0.5)
        self.step(gh, "contenedor", 0.80, mc.pose, bx, by, bz + self.approach, -math.pi / 2, self.base_frame, False, v)
        self.step(gh, "soltar", 0.90, mc.gripper, 1.0)
        self.step(gh, "retorno", 1.00, mc.named, self.observe, v)
        return True, f"{lab} depositada en ({bx:.2f}, {by:.2f})"

    def execute(self, gh):
        req = gh.request
        place = None
        if req.use_place_pose:
            p = req.place_pose.pose.position
            place = (p.x, p.y, p.z)
        try:
            ok, msg = self.pick_and_place(req.label, place, gh)
        except RuntimeError as e:
            ok, msg = False, str(e)
        res = PickPlace.Result(success=ok, message=msg)
        (gh.succeed if ok else gh.abort)()
        return res

    def auto_loop(self):
        if not self.dry and not self.mc.wait(60.0):
            self.get_logger().error("motion_server no disponible")
            return
        while rclpy.ok():
            try:
                ok, msg = self.pick_and_place("")
            except RuntimeError as e:
                ok, msg = False, str(e)
            self.get_logger().info(f"Ciclo: {ok} - {msg}")
            if not ok:
                time.sleep(3.0)


def main():
    rclpy.init()
    node = PickPlaceNode()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    rclpy.try_shutdown()
