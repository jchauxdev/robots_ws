"""App web de logistica (Flask + rclpy): pedidos, estado del robot y del cobot, mapa y teleoperacion.

  ros2 run mr_logistics web_app --ros-args -p port:=8080
  Abra http://<IP del equipo>:8080 desde el PC o el celular (misma red).

La App NO controla el robot directamente: llama servicios ROS 2 del gestor de misiones
(/logistics/submit_order, /logistics/cancel_order, /logistics/confirm_delivery) y del gestor de estaciones
(/mr/go_to, /mr/cancel). La teleoperacion publica cmd_vel solo si no hay un pedido en curso.
Seguridad: pensada para la red local del laboratorio. Si se fija el parametro 'access_token', cada
peticion /api debe incluir ?token=... o la cabecera X-Token.
"""
import io
import math
import threading
import time

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from flask import Flask, Response, jsonify, render_template, request
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from mr_interfaces.msg import Order, OrderStatus
from mr_interfaces.srv import CancelOrder, GoTo, SubmitOrder
from mr_navigation import stations as st

ACTIVE = {'HACIA_COBOT', 'ESPERANDO_CARGA', 'CARGANDO', 'HACIA_DESTINO', 'ENTREGADO', 'REGRESANDO'}


class Bridge(Node):
    """Lado ROS 2 de la App: guarda el ultimo estado conocido y ofrece llamadas sincronas a servicios."""

    def __init__(self):
        super().__init__('logistics_web')
        p = self.declare_parameter
        self.port = int(p('port', 8080).value)
        self.host = p('host', '0.0.0.0').value
        self.token = p('access_token', '').value
        self.products = list(p('products', ['rojo', 'verde', 'azul', 'aruco_7', 'apple', 'orange', 'banana']).value)
        self.max_v = float(p('teleop_max_linear', 0.25).value)
        self.max_w = float(p('teleop_max_angular', 1.0).value)
        _, self.stations = st.load(p('stations_file', 'estaciones.yaml').value)
        self.pickup = p('pickup_station', 'cobot').value
        self.cb = ReentrantCallbackGroup()
        self.lock = threading.Lock()
        self.orders, self.nav_state, self.cobot_status = {}, '-', '-'
        self.pose, self.battery, self.map_msg, self.last_teleop = None, None, None, 0.0
        latched = QoSProfile(depth=50, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OrderStatus, '/logistics/order_status', self.on_status, latched)
        self.create_subscription(String, '/mr/nav_state', lambda m: setattr(self, 'nav_state', m.data), 10)
        self.create_subscription(String, '/cobot/status', lambda m: setattr(self, 'cobot_status', m.data), 10)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.on_pose, 10)
        self.create_subscription(DiagnosticArray, 'diagnostics', self.on_diag, 10)
        self.create_subscription(OccupancyGrid, 'map', lambda m: setattr(self, 'map_msg', m), map_qos)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        # AMCL solo publica amcl_pose al moverse: como respaldo se consulta la TF map -> base_footprint
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cli = {
            'submit': self.create_client(SubmitOrder, '/logistics/submit_order', callback_group=self.cb),
            'cancel': self.create_client(CancelOrder, '/logistics/cancel_order', callback_group=self.cb),
            'confirm': self.create_client(Trigger, '/logistics/confirm_delivery', callback_group=self.cb),
            'goto': self.create_client(GoTo, '/mr/go_to', callback_group=self.cb),
            'stop': self.create_client(Trigger, '/mr/cancel', callback_group=self.cb),
        }

    # -------------------------------------------------------------- suscripciones
    def on_status(self, m):
        with self.lock:
            self.orders[m.id] = {'id': m.id, 'state': m.state, 'detail': m.detail, 'product': m.product,
                                 'quantity': m.quantity, 'destination': m.destination,
                                 't': m.stamp.sec + m.stamp.nanosec * 1e-9}

    def on_pose(self, m):
        q = m.pose.pose.orientation
        self.pose = (m.pose.pose.position.x, m.pose.pose.position.y, 2 * math.atan2(q.z, q.w))

    def on_diag(self, m):
        for s in m.status:
            if s.name.startswith('mr_base'):
                for kv in s.values:
                    if kv.key == 'bateria_principal_V':
                        self.battery = float(kv.value)

    # -------------------------------------------------------------- servicios
    def call(self, key, req, timeout=5.0):
        c = self.cli[key]
        if not c.wait_for_service(timeout_sec=2.0):
            return None
        ev, box = threading.Event(), {}
        f = c.call_async(req)
        f.add_done_callback(lambda fut: (box.__setitem__('r', fut.result()), ev.set()))
        return box.get('r') if ev.wait(timeout) else None

    def current_pose(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            q = t.transform.rotation
            return (t.transform.translation.x, t.transform.translation.y, 2 * math.atan2(q.z, q.w))
        except Exception:  # noqa: BLE001
            return self.pose

    def busy(self):
        with self.lock:
            return any(o['state'] in ACTIVE for o in self.orders.values())

    def state(self):
        with self.lock:
            orders = sorted(self.orders.values(), key=lambda o: o['id'], reverse=True)
        return {'orders': orders, 'nav_state': self.nav_state, 'cobot_status': self.cobot_status,
                'pose': self.current_pose(), 'battery': self.battery, 'busy': self.busy(),
                'products': self.products, 'stations': [n for n in self.stations if n != self.pickup],
                'all_stations': list(self.stations)}

    # -------------------------------------------------------------- mapa
    def map_png(self):
        from PIL import Image, ImageDraw
        m = self.map_msg
        if m is None:
            img = Image.new('RGB', (400, 300), (60, 60, 60))
            ImageDraw.Draw(img).text((10, 140), 'Sin mapa (/map): lance navigation.launch.py', fill=(255, 255, 255))
        else:
            w, h, res = m.info.width, m.info.height, m.info.resolution
            ox, oy = m.info.origin.position.x, m.info.origin.position.y
            g = np.array(m.data, dtype=np.int16).reshape(h, w)
            rgb = np.full((h, w, 3), 205, np.uint8)                 # desconocido
            rgb[(g >= 0) & (g < 25)] = 254                          # libre
            rgb[(g >= 25) & (g <= 65)] = 150
            rgb[g > 65] = 0                                         # ocupado
            img = Image.fromarray(rgb[::-1])                        # fila 0 del mapa = y minima
            k = max(1, int(600 / max(w, h)))
            img = img.resize((w * k, h * k), Image.NEAREST)
            d = ImageDraw.Draw(img)

            def px(x, y):
                return (x - ox) / res * k, (h - (y - oy) / res) * k

            for s in self.stations.values():
                u, v = px(s.x, s.y)
                d.ellipse([u - 5, v - 5, u + 5, v + 5], fill=(57, 169, 0))
                tw = d.textlength(s.name) if hasattr(d, 'textlength') else 6 * len(s.name)
                tx = u + 7 if u + 7 + tw < img.width else u - 7 - tw      # evita cortar el nombre en el borde
                d.text((tx, v - 7), s.name, fill=(0, 80, 0))
            pose = self.current_pose()
            if pose:
                x, y, th = pose
                u, v = px(x, y)
                d.ellipse([u - 7, v - 7, u + 7, v + 7], outline=(0, 0, 200), width=3)
                d.line([u, v, u + 14 * math.cos(th), v - 14 * math.sin(th)], fill=(0, 0, 200), width=3)
        buf = io.BytesIO()
        img.save(buf, 'PNG')
        return buf.getvalue()


def create_app(node: Bridge):
    app = Flask(__name__)

    @app.before_request
    def check_token():
        if node.token and request.path.startswith('/api'):
            if request.args.get('token') != node.token and request.headers.get('X-Token') != node.token:
                return jsonify(ok=False, message='token invalido'), 401
        return None

    @app.route('/')
    def index():
        return render_template('index.html', token=node.token)

    @app.route('/teleop')
    def teleop():
        return render_template('teleop.html', token=node.token)

    @app.route('/api/state')
    def api_state():
        return jsonify(node.state())

    @app.route('/api/map.png')
    def api_map():
        return Response(node.map_png(), mimetype='image/png')

    @app.route('/api/orders', methods=['POST'])
    def api_order():
        d = request.get_json(force=True, silent=True) or {}
        try:
            qty = int(d.get('quantity', 1))
        except (TypeError, ValueError):
            return jsonify(ok=False, message='Cantidad invalida'), 400
        req = SubmitOrder.Request(order=Order(product=str(d.get('product', '')), quantity=qty,
                                              destination=str(d.get('destination', ''))))
        r = node.call('submit', req)
        if r is None:
            return jsonify(ok=False, message='Gestor de misiones no disponible'), 503
        return jsonify(ok=r.accepted, id=r.id, message=r.message), (200 if r.accepted else 400)

    @app.route('/api/orders/<oid>/cancel', methods=['POST'])
    def api_cancel(oid):
        r = node.call('cancel', CancelOrder.Request(id=oid))
        return jsonify(ok=bool(r and r.success), message=r.message if r else 'sin respuesta')

    @app.route('/api/confirm', methods=['POST'])
    def api_confirm():
        r = node.call('confirm', Trigger.Request())
        return jsonify(ok=bool(r and r.success), message=r.message if r else 'sin respuesta')

    @app.route('/api/goto', methods=['POST'])
    def api_goto():
        if node.busy():
            return jsonify(ok=False, message='Hay un pedido en curso'), 409
        name = (request.get_json(force=True, silent=True) or {}).get('name', '')
        r = node.call('goto', GoTo.Request(name=name))
        return jsonify(ok=bool(r and r.accepted), message=r.message if r else 'sin respuesta')

    @app.route('/api/stop', methods=['POST'])
    def api_stop():
        node.cmd_pub.publish(Twist())
        r = node.call('stop', Trigger.Request())
        return jsonify(ok=True, message=r.message if r else 'parada enviada')

    @app.route('/api/cmd_vel', methods=['POST'])
    def api_cmd():
        if node.busy():
            return jsonify(ok=False, message='Teleoperacion bloqueada: pedido en curso'), 409
        d = request.get_json(force=True, silent=True) or {}
        t = Twist()
        t.linear.x = max(-node.max_v, min(node.max_v, float(d.get('v', 0.0))))
        t.angular.z = max(-node.max_w, min(node.max_w, float(d.get('w', 0.0))))
        node.cmd_pub.publish(t)
        node.last_teleop = time.time()
        return jsonify(ok=True)

    return app


def main():
    rclpy.init()
    node = Bridge()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    app = create_app(node)
    node.get_logger().info(f'App logistica en http://{node.host}:{node.port}')
    try:
        app.run(host=node.host, port=node.port, threaded=True, use_reloader=False)
    except KeyboardInterrupt:
        pass
    finally:
        ex.shutdown()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
