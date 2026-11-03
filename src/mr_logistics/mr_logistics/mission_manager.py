"""Gestor de misiones logisticas: robot movil + JetCobot.

Flujo de cada pedido (maquina de estados, un pedido a la vez, cola FIFO):
  RECIBIDO -> EN_COLA -> HACIA_COBOT -> ESPERANDO_CARGA -> CARGANDO -> HACIA_DESTINO -> ENTREGADO
           -> (REGRESANDO) -> COMPLETADO            (o FALLIDO / CANCELADO)

  1. La App web llama /logistics/submit_order. El pedido se publica en /logistics/orders para que el
     cobot lo conozca de antemano (verifica que el producto este en su mesa).
  2. El robot navega a la estacion 'cobot' (accion /mr/go_to_station del station_manager).
  3. Al llegar AVISA al cobot enviando la meta /cobot/load_product (product, quantity).
  4. El cobot detecta el producto y lo deja en la bandeja; al terminar responde (resultado de la accion):
     ese es el AVISO del cobot al robot para que arranque.
  5. El robot navega a la estacion destino, espera la descarga (tiempo o boton "Recibido") y regresa a casa.

Servicios: /logistics/submit_order, /logistics/cancel_order, /logistics/confirm_delivery (Trigger)
Topicos  : /logistics/orders (Order), /logistics/order_status (OrderStatus, latched x50)
"""
import queue
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_srvs.srv import Trigger

from mr_interfaces.action import GoToStation, LoadProduct
from mr_interfaces.msg import Order, OrderStatus
from mr_interfaces.srv import CancelOrder, SubmitOrder
from mr_navigation import stations as st

from .orders import OrderBook


class Cancelled(Exception):
    pass


class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')
        p = self.declare_parameter
        self.pickup = p('pickup_station', 'cobot').value
        self.home = p('home_station', 'inicio').value
        self.unload_time = float(p('unload_time', 8.0).value)
        self.return_home = bool(p('return_home', True).value)
        self.load_timeout = float(p('load_timeout', 300.0).value)
        self.nav_retries = int(p('nav_retries', 1).value)
        products = list(p('products', ['rojo', 'verde', 'azul', 'aruco_7', 'apple', 'orange', 'banana']).value)
        _, stations = st.load(p('stations_file', 'estaciones.yaml').value)
        self.book = OrderBook(products, stations.keys(), self.pickup, int(p('max_quantity', 5).value))

        self.cb = ReentrantCallbackGroup()
        latched = QoSProfile(depth=50, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(OrderStatus, '/logistics/order_status', latched)
        self.order_pub = self.create_publisher(Order, '/logistics/orders', latched)
        self.nav = ActionClient(self, GoToStation, '/mr/go_to_station', callback_group=self.cb)
        self.cobot = ActionClient(self, LoadProduct, '/cobot/load_product', callback_group=self.cb)
        self.create_service(SubmitOrder, '/logistics/submit_order', self.on_submit, callback_group=self.cb)
        self.create_service(CancelOrder, '/logistics/cancel_order', self.on_cancel, callback_group=self.cb)
        self.create_service(Trigger, '/logistics/confirm_delivery', self.on_confirm, callback_group=self.cb)
        self.q = queue.Queue()
        self.current = None
        self.cancel_flags = set()
        self.delivered = threading.Event()
        self.goal_handle = None
        threading.Thread(target=self.worker, daemon=True).start()
        self.get_logger().info(f'Gestor listo. Productos: {products}. Estaciones: {sorted(stations)}')

    # ------------------------------------------------------------------ estado
    def set_state(self, oid, state, detail=''):
        rec = self.book.set_state(oid, state, detail)
        msg = OrderStatus(id=rec.id, state=rec.state, detail=rec.detail, product=rec.product,
                          quantity=rec.quantity, destination=rec.destination)
        msg.stamp = self.get_clock().now().to_msg()
        self.status_pub.publish(msg)
        self.get_logger().info(f'[{oid}] {state} {detail}')

    # ------------------------------------------------------------------ servicios
    def on_submit(self, req, res):
        o = req.order
        rec, text = self.book.create(o.product.strip(), int(o.quantity), o.destination.strip(), o.id.strip())
        res.accepted, res.message = rec is not None, text
        if rec is None:
            return res
        res.id = rec.id
        self.set_state(rec.id, 'EN_COLA', f'posicion {self.q.qsize() + (1 if self.current else 0) + 1}')
        om = Order(id=rec.id, product=rec.product, quantity=rec.quantity, pickup_station=self.pickup,
                   destination=rec.destination)
        om.stamp = self.get_clock().now().to_msg()
        self.order_pub.publish(om)
        self.q.put(rec.id)
        return res

    def on_cancel(self, req, res):
        rec = self.book.get(req.id)
        if rec is None or rec.state in ('COMPLETADO', 'FALLIDO', 'CANCELADO'):
            res.success, res.message = False, f'Pedido {req.id} inexistente o ya terminado'
            return res
        self.cancel_flags.add(req.id)
        if self.current == req.id and self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        if self.current != req.id:
            self.set_state(req.id, 'CANCELADO', 'cancelado antes de iniciar')
        res.success, res.message = True, f'Cancelando {req.id}'
        return res

    def on_confirm(self, _req, res):
        self.delivered.set()
        res.success, res.message = True, 'Entrega confirmada'
        return res

    # ------------------------------------------------------------------ acciones
    def _run_action(self, client, goal, oid, feedback=None, timeout=600.0):
        if not client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError(f'Servidor de accion {client._action_name} no disponible')
        ev = threading.Event()
        box = {}
        f = client.send_goal_async(goal, feedback_callback=feedback)
        f.add_done_callback(lambda fut: (box.__setitem__('gh', fut.result()), ev.set()))
        if not ev.wait(10.0) or not box['gh'].accepted:
            raise RuntimeError('Meta rechazada')
        self.goal_handle = box['gh']
        ev.clear()
        rf = self.goal_handle.get_result_async()
        rf.add_done_callback(lambda fut: ev.set())
        t0 = time.time()
        while not ev.wait(0.2):
            if oid in self.cancel_flags:
                self.goal_handle.cancel_goal_async()
            if time.time() - t0 > timeout:
                self.goal_handle.cancel_goal_async()
                raise RuntimeError('Tiempo agotado')
        self.goal_handle = None
        if oid in self.cancel_flags:
            raise Cancelled()
        return rf.result().result

    def go(self, oid, station, state):
        for attempt in range(self.nav_retries + 1):
            self.set_state(oid, state, f'navegando a {station}' + (f' (reintento {attempt})' if attempt else ''))
            res = self._run_action(self.nav, GoToStation.Goal(name=station), oid)
            if res.success:
                return res
            self.get_logger().warn(f'[{oid}] navegacion a {station} fallo: {res.message}')
        raise RuntimeError(f'No se pudo llegar a {station}: {res.message}')

    # ------------------------------------------------------------------ ciclo
    def worker(self):
        while rclpy.ok():
            try:
                oid = self.q.get(timeout=0.5)
            except queue.Empty:
                continue
            rec = self.book.get(oid)
            if rec.state == 'CANCELADO':
                continue
            self.current = oid
            try:
                self.go(oid, self.pickup, 'HACIA_COBOT')
                self.set_state(oid, 'ESPERANDO_CARGA', 'robot en la estacion del cobot: avisando al cobot')

                last = {'stage': None}

                def fb(m):
                    f = m.feedback
                    if f.stage != last['stage']:          # publica solo cuando cambia la etapa
                        last['stage'] = f.stage
                        self.set_state(oid, 'CARGANDO', f'{f.stage} ({f.loaded}/{rec.quantity}, '
                                                        f'{f.progress * 100:.0f} %)')

                res = self._run_action(self.cobot, LoadProduct.Goal(order_id=oid, product=rec.product,
                                                                    quantity=rec.quantity), oid, fb,
                                       timeout=self.load_timeout)
                if not res.success:
                    raise RuntimeError(f'Cobot: {res.message} (cargadas {res.loaded}/{rec.quantity})')
                self.set_state(oid, 'CARGANDO', f'cobot termino: {res.message}. Saliendo hacia {rec.destination}')
                r = self.go(oid, rec.destination, 'HACIA_DESTINO')
                self.delivered.clear()
                self.set_state(oid, 'ENTREGADO', f'en {rec.destination} (error {r.final_distance * 100:.1f} cm); '
                                                 f'esperando descarga {self.unload_time:.0f} s o confirmacion')
                self.delivered.wait(self.unload_time)
                if self.return_home and self.q.empty():
                    self.go(oid, self.home, 'REGRESANDO')
                self.set_state(oid, 'COMPLETADO', f'{rec.quantity} x {rec.product} entregado en {rec.destination}')
            except Cancelled:
                self.set_state(oid, 'CANCELADO', 'cancelado durante la ejecucion')
            except Exception as e:  # noqa: BLE001
                self.set_state(oid, 'FALLIDO', str(e))
            finally:
                self.current = None
                self.cancel_flags.discard(oid)


def main():
    rclpy.init()
    node = MissionManager()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
