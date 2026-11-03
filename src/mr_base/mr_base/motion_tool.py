"""Movimientos medidos y calibracion de odometria (DA-12.2).

Uso:
  ros2 run mr_base mover avanzar 1.0             # avanza 1 m segun la odometria y se detiene
  ros2 run mr_base mover girar 90                # gira 90 grados en el sitio
  ros2 run mr_base mover lineal 1.0              # prueba de calibracion lineal (pide la medida real)
  ros2 run mr_base mover angular 360             # prueba de calibracion angular (pide el angulo real)
  ros2 run mr_base mover cuadrado 1.0 --sentido ccw   # trayectoria cuadrada (prueba UMBmark)

Opciones:  --vel 0.15  --wvel 0.6  --medido X (no pregunta)  --config ruta.yaml (escribe el resultado)
           --verdad /sim/ground_truth  (solo simulacion: toma la medida "real" del emulador)

El control es en lazo cerrado sobre /odom: perfil trapezoidal de velocidad, correccion de rumbo con un
controlador P y tolerancia final de 2 mm / 0.5 grados. El robot se detiene cuando la ODOMETRIA indica que
llego; la diferencia con la cinta metrica es justamente el error de calibracion.
"""
import argparse
import math
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters

from .kinematics import corrected_ticks_per_meter, corrected_wheel_separation, normalize_angle


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Mover(Node):
    def __init__(self, truth_topic=None):
        super().__init__('mr_mover')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom = None
        self.truth = None
        self.yaw_unwrapped = None
        self._last_yaw = None
        self.cv = threading.Condition()
        self.create_subscription(Odometry, 'odom', self.on_odom, 20)
        if truth_topic:
            self.create_subscription(Odometry, truth_topic, self.on_truth, 20)

    def on_odom(self, m):
        yaw = yaw_of(m.pose.pose.orientation)
        if self._last_yaw is None:
            self.yaw_unwrapped = yaw
        else:
            self.yaw_unwrapped += normalize_angle(yaw - self._last_yaw)
        self._last_yaw = yaw
        with self.cv:
            self.odom = (m.pose.pose.position.x, m.pose.pose.position.y, yaw, self.yaw_unwrapped,
                         m.twist.twist.linear.x, m.twist.twist.angular.z)
            self.cv.notify_all()

    def on_truth(self, m):
        self.truth = (m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(m.pose.pose.orientation))

    def wait_odom(self, timeout=5.0):
        t0 = time.time()
        while self.odom is None and time.time() - t0 < timeout:
            time.sleep(0.05)
        if self.odom is None:
            raise RuntimeError('No llega /odom: lance el nodo mr_base (o el emulador)')
        return self.odom

    def send(self, v, w):
        t = Twist()
        t.linear.x, t.angular.z = float(v), float(w)
        self.pub.publish(t)

    def stop(self, settle=0.8):
        t0 = time.time()
        while time.time() - t0 < settle:
            self.send(0.0, 0.0)
            time.sleep(0.05)

    # ---------------------------------------------------------------- primitivas
    def straight(self, dist, vmax=0.15, acc=0.3, tol=0.001, latency=0.15):
        """Avanza 'dist' metros (negativo = reversa) segun la odometria.

        Perfil trapezoidal con frenado anticipado: se descuenta la distancia que el robot recorre durante la
        latencia del lazo (lectura de encoders + envio de la orden, ~0.1-0.15 s) para no pasarse de la meta.
        """
        x0, y0, th0, *_ = self.wait_odom()
        sgn = 1.0 if dist >= 0 else -1.0
        target = abs(dist)
        v = 0.0
        dt = 0.02
        while rclpy.ok():
            x, y, th, *_ = self.odom
            s = (x - x0) * math.cos(th0) + (y - y0) * math.sin(th0)   # avance proyectado
            rem = target - sgn * s - v * latency                       # lo que falta descontando la latencia
            if rem <= tol:
                break
            v_des = min(vmax, max(0.02, math.sqrt(2 * acc * rem)))
            v = min(v_des, v + acc * dt)
            w = 2.0 * normalize_angle(th0 - th)                         # mantiene el rumbo
            self.send(sgn * v, w)
            time.sleep(dt)
        self.stop()
        x, y, th, *_ = self.odom
        return (x - x0) * math.cos(th0) + (y - y0) * math.sin(th0)

    def rotate(self, angle_rad, wmax=0.6, acc=1.0, tol=math.radians(0.2), latency=0.15):
        *_, yu0, _, _ = self.wait_odom()
        sgn = 1.0 if angle_rad >= 0 else -1.0
        target = abs(angle_rad)
        w = 0.0
        dt = 0.02
        while rclpy.ok():
            rem = target - sgn * (self.odom[3] - yu0) - w * latency
            if rem <= tol:
                break
            w_des = min(wmax, max(0.08, math.sqrt(2 * acc * rem)))
            w = min(w_des, w + acc * dt)
            self.send(0.0, sgn * w)
            time.sleep(dt)
        self.stop()
        return self.odom[3] - yu0


def ask_float(prompt):
    while True:
        try:
            return float(input(prompt).replace(',', '.'))
        except ValueError:
            print('Valor no valido, intente de nuevo.')


def read_base_params(node, names):
    cli = node.create_client(GetParameters, 'mr_base/get_parameters')
    if not cli.wait_for_service(timeout_sec=3.0):
        return {}
    fut = cli.call_async(GetParameters.Request(names=names))
    t0 = time.time()
    while not fut.done() and time.time() - t0 < 3.0:
        time.sleep(0.05)
    if not fut.done():
        return {}
    out = {}
    for n, v in zip(names, fut.result().values):
        out[n] = v.double_value if v.type == 3 else (v.integer_value if v.type == 2 else None)
    return out


def update_yaml(path, key, value):
    import yaml
    with open(path) as f:
        data = yaml.safe_load(f)
    root = next(k for k in data if k.rstrip('/').split('/')[-1] == 'mr_base')   # 'mr_base' o '/**/mr_base'
    data[root]['ros__parameters'][key] = float(round(value, 4))
    with open(path, 'w') as f:
        yaml.safe_dump(data, f, sort_keys=False, allow_unicode=True)


def main(argv=None):
    ap = argparse.ArgumentParser(prog='mover', description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('modo', choices=['avanzar', 'girar', 'lineal', 'angular', 'cuadrado'])
    ap.add_argument('valor', type=float, help='metros (avanzar/lineal/cuadrado) o grados (girar/angular)')
    ap.add_argument('--vel', type=float, default=0.15, help='velocidad lineal maxima [m/s]')
    ap.add_argument('--wvel', type=float, default=0.6, help='velocidad angular maxima [rad/s]')
    ap.add_argument('--medido', type=float, default=None, help='medida real (m o grados); evita la pregunta')
    ap.add_argument('--config', default=None, help='YAML de mr_base a actualizar con el valor calibrado')
    ap.add_argument('--verdad', default=None, help='topico de "verdad terreno" (solo emulador/simulacion)')
    ap.add_argument('--sentido', choices=['cw', 'ccw'], default='ccw')
    args, ros_args = ap.parse_known_args(argv if argv is not None else sys.argv[1:])

    rclpy.init(args=ros_args)
    node = Mover(args.verdad)
    ex = rclpy.executors.MultiThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    try:
        node.wait_odom()
        if args.verdad:
            t0 = time.time()
            while node.truth is None and time.time() - t0 < 5.0:
                time.sleep(0.05)
        time.sleep(0.3)
        truth0 = node.truth
        if args.modo in ('avanzar', 'lineal'):
            d_odo = node.straight(args.valor, args.vel)
            print(f'Odometria: {d_odo:.4f} m (objetivo {args.valor:.3f} m)')
            if args.modo == 'lineal':
                if args.medido is not None:
                    d_real = args.medido
                elif args.verdad and node.truth and truth0:
                    d_real = ((node.truth[0] - truth0[0]) * math.cos(truth0[2]) +
                              (node.truth[1] - truth0[1]) * math.sin(truth0[2]))
                    print(f'Distancia real (verdad terreno): {d_real:.4f} m')
                else:
                    d_real = ask_float('Distancia REAL medida con cinta [m]: ')
                d_odo_abs, d_real_abs = abs(d_odo), abs(d_real)
                cur = read_base_params(node, ['ticks_per_meter', 'encoder_cpr', 'gear_ratio', 'wheel_diameter'])
                tpm = cur.get('ticks_per_meter') or 0.0
                if not tpm and all(cur.get(k) for k in ('encoder_cpr', 'gear_ratio', 'wheel_diameter')):
                    tpm = cur['encoder_cpr'] * cur['gear_ratio'] / (math.pi * cur['wheel_diameter'])
                err = 100.0 * (d_odo_abs - d_real_abs) / d_real_abs
                print(f'Error: {1000 * (d_odo_abs - d_real_abs):+.1f} mm ({err:+.2f} %)')
                if tpm:
                    new = corrected_ticks_per_meter(tpm, d_odo_abs, d_real_abs)
                    print(f'ticks_per_meter: {tpm:.1f} -> {new:.1f}')
                    if args.config:
                        update_yaml(args.config, 'ticks_per_meter', new)
                        print(f'Actualizado {args.config}. Reinicie mr_base y repita la prueba.')
                else:
                    print('No se pudieron leer los parametros de /mr_base (esta corriendo el nodo?).')
        elif args.modo in ('girar', 'angular'):
            th_odo = node.rotate(math.radians(args.valor), args.wvel)
            print(f'Odometria: {math.degrees(th_odo):.2f} grados (objetivo {args.valor:.1f})')
            if args.modo == 'angular':
                if args.medido is not None:
                    th_real = math.radians(args.medido)
                elif args.verdad and node.truth and truth0:
                    # giro real = giro de odometria + diferencia angular (valida si el error < 180 grados)
                    th_real = th_odo + normalize_angle((node.truth[2] - truth0[2]) - th_odo)
                    print(f'Giro real (verdad terreno): {math.degrees(th_real):.2f} grados')
                else:
                    th_real = math.radians(ask_float('Angulo REAL girado [grados]: '))
                cur = read_base_params(node, ['wheel_separation'])
                b = cur.get('wheel_separation')
                print(f'Error: {math.degrees(th_odo - th_real):+.2f} grados')
                if b:
                    new = corrected_wheel_separation(b, th_odo, th_real)
                    print(f'wheel_separation: {b:.4f} -> {new:.4f} m')
                    if args.config:
                        update_yaml(args.config, 'wheel_separation', new)
                        print(f'Actualizado {args.config}. Reinicie mr_base y repita la prueba.')
        else:  # cuadrado
            sgn = 1.0 if args.sentido == 'ccw' else -1.0
            for i in range(4):
                node.straight(args.valor, args.vel)
                node.rotate(sgn * math.pi / 2, args.wvel)
            x, y, th, *_ = node.odom
            print(f'Pose final segun odometria: x={x:.3f} y={y:.3f} th={math.degrees(th):.1f} grados')
            if node.truth and truth0:
                dx, dy = node.truth[0] - truth0[0], node.truth[1] - truth0[1]
                print(f'Error de cierre real: dx={dx:+.3f} m dy={dy:+.3f} m')
            print('Mida en el piso la posicion final respecto a la marca de inicio (UMBmark, DA-12.2).')
    except KeyboardInterrupt:
        pass
    finally:
        node.stop(0.3)
        ex.shutdown(timeout_sec=1.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
