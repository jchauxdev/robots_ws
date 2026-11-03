"""Estacion del JetCobot para la logistica: recibe pedidos y carga el producto en la bandeja del robot movil.

Corre en el equipo del cobot (con los paquetes jc_* compilados en robots_ws y su celda en marcha: real.launch.py / sim / mock +
vision + pick_place_node).

  * /logistics/orders (mr_interfaces/Order): el cobot conoce el pedido ANTES de que llegue el robot y
    verifica con su vision si hay suficientes piezas de ese producto en la mesa (/cobot/status).
  * Accion /cobot/load_product (mr_interfaces/LoadProduct): el robot movil la envia al llegar ("aviso").
      1. Busca la bandeja del robot: marcador ArUco 'tray_label' (p. ej. aruco_10) pegado en la bandeja;
         si no lo ve, usa la pose nominal 'tray_pose' (robot atracado en la estacion 'cobot').
      2. Repite 'quantity' veces la accion /jc/pick_place (jc_apps) con label=product y destino = bandeja,
         separando cada unidad 'spacing' metros para no apilarlas.
      3. Responde el resultado: ese es el AVISO al robot movil para que arranque.
"""
import math
import statistics
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from jc_interfaces.action import PickPlace
from jc_interfaces.msg import DetectedObjectArray
from mr_interfaces.action import LoadProduct
from mr_interfaces.msg import Order


class CobotStation(Node):
    def __init__(self):
        super().__init__('cobot_station')
        p = self.declare_parameter
        self.frame = p('base_frame', 'world').value
        self.tray = [float(v) for v in p('tray_pose', [0.02, -0.17, 0.05]).value]   # x, y, z en base_frame
        self.tray_label = p('tray_label', 'aruco_10').value
        self.tray_z_offset = float(p('tray_z_offset', 0.03).value)    # altura de deposito sobre el marcador
        self.spacing = float(p('spacing', 0.035).value)
        self.max_tray_shift = float(p('max_tray_shift', 0.08).value)  # m: rechaza detecciones absurdas
        self.dry_run = bool(p('dry_run', False).value)
        self.cb = ReentrantCallbackGroup()
        self.lock = threading.Lock()
        self.dets = []                  # (t, DetectedObjectArray)
        self.create_subscription(DetectedObjectArray, '/jc/detections', self.on_det, 10, callback_group=self.cb)
        self.create_subscription(Order, '/logistics/orders', self.on_order, 10, callback_group=self.cb)
        self.status = self.create_publisher(String, '/cobot/status', 10)
        self.pick = ActionClient(self, PickPlace, '/jc/pick_place', callback_group=self.cb)
        ActionServer(self, LoadProduct, '/cobot/load_product', self.execute, callback_group=self.cb,
                     cancel_callback=lambda _g: CancelResponse.ACCEPT)
        self.say(f'Estacion cobot lista (bandeja nominal {self.tray}, marcador {self.tray_label})')

    def say(self, text):
        self.status.publish(String(data=text))
        self.get_logger().info(text)

    def on_det(self, m):
        now = time.monotonic()
        with self.lock:
            self.dets.append((now, m))
            self.dets = [(t, d) for t, d in self.dets if now - t < 3.0]

    def visible(self, label, window=1.0):
        """Lista de poses (x, y, z) del objeto 'label' vistas en el ultimo 'window' s (deteccion mas reciente)."""
        now = time.monotonic()
        with self.lock:
            recent = [d for t, d in self.dets if now - t < window]
        if not recent:
            return []
        return [(o.pose.position.x, o.pose.position.y, o.pose.position.z)
                for o in recent[-1].objects if o.label == label and o.pose_valid]

    def on_order(self, o):
        n = len(self.visible(o.product, 2.0))
        txt = f'Pedido {o.id}: {o.quantity} x {o.product} -> {o.destination}. En la mesa veo {n}.'
        if n < o.quantity:
            txt += ' ATENCION: faltan piezas (reponga o revise la vision).'
        self.say(txt)

    def tray_target(self):
        t0, xs, ys, zs = time.monotonic(), [], [], []
        while time.monotonic() - t0 < 2.0:
            for x, y, z in self.visible(self.tray_label, 0.3):
                xs.append(x), ys.append(y), zs.append(z)
            time.sleep(0.1)
        if len(xs) >= 3:
            x, y, z = statistics.median(xs), statistics.median(ys), statistics.median(zs) + self.tray_z_offset
            if math.hypot(x - self.tray[0], y - self.tray[1]) <= self.max_tray_shift:
                return (x, y, z), f'bandeja localizada con {self.tray_label}'
            self.get_logger().warn(f'{self.tray_label} lejos de la pose nominal; se ignora')
        return tuple(self.tray), 'bandeja en pose nominal (sin marcador)'

    def execute(self, gh):
        req = gh.request
        res = LoadProduct.Result(success=False, loaded=0)
        self.say(f'Robot atracado: cargando pedido {req.order_id} ({req.quantity} x {req.product})')
        (bx, by, bz), how = self.tray_target()
        self.say(how)
        if not self.dry_run and not self.pick.wait_for_server(timeout_sec=10.0):
            res.message = 'pick_place_node (/jc/pick_place) no disponible'
            gh.abort()
            return res
        for i in range(req.quantity):
            if gh.is_cancel_requested:
                res.message = 'cancelado'
                gh.canceled()
                return res
            off = (i - (req.quantity - 1) / 2.0) * self.spacing
            gh.publish_feedback(LoadProduct.Feedback(stage=f'pieza {i + 1}: recoger y dejar en bandeja',
                                                     loaded=i, progress=i / req.quantity))
            if self.dry_run:
                time.sleep(1.0)
                ok, msg = True, 'simulado'
            else:
                ok, msg = self.pick_one(req.product, bx + off, by, bz, gh)
            if not ok:
                res.message = f'fallo en la pieza {i + 1}: {msg}'
                self.say(res.message)
                gh.abort()
                return res
            res.loaded = i + 1
        gh.publish_feedback(LoadProduct.Feedback(stage='carga completa', loaded=res.loaded, progress=1.0))
        res.success, res.message = True, f'{res.loaded} x {req.product} en la bandeja'
        self.say(f'Pedido {req.order_id} cargado: el robot puede salir')
        gh.succeed()
        return res

    def pick_one(self, label, x, y, z, gh):
        goal = PickPlace.Goal(label=label, use_place_pose=True)
        goal.place_pose = PoseStamped()
        goal.place_pose.header.frame_id = self.frame
        goal.place_pose.pose.position.x, goal.place_pose.pose.position.y = x, y
        goal.place_pose.pose.position.z = z
        goal.place_pose.pose.orientation.w = 1.0
        ev, box = threading.Event(), {}
        f = self.pick.send_goal_async(goal)
        f.add_done_callback(lambda fut: (box.__setitem__('gh', fut.result()), ev.set()))
        if not ev.wait(10.0) or not box['gh'].accepted:
            return False, 'meta de pick_place rechazada'
        ev.clear()
        rf = box['gh'].get_result_async()
        rf.add_done_callback(lambda fut: ev.set())
        while not ev.wait(0.2):
            if gh.is_cancel_requested:
                box['gh'].cancel_goal_async()
        r = rf.result().result
        return r.success, r.message


def main():
    rclpy.init()
    node = CobotStation()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
