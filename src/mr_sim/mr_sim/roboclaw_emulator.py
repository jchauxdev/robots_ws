"""Emulador de RoboClaw 2x15A + motores Pololu + robot diferencial (para practicar y validar sin hardware).

Crea un puerto serie virtual (pseudo-terminal) y lo enlaza en ~link (por defecto /tmp/roboclaw_emu). El nodo
mr_base se conecta a ese puerto exactamente igual que al RoboClaw real: mismo protocolo Packet Serial con CRC16.

Modelo fisico (100 Hz):
  * Cada motor sigue la velocidad pedida (QPPS) con la rampa de aceleracion del comando 40 y una constante de
    tiempo de primer orden 'motor_tau' (aproxima el PID de velocidad sintonizado con Autotune).
  * Encoders de cuadratura: cuentas = vueltas de rueda * encoder_cpr * gear_ratio.
  * El robot se mueve con la geometria REAL ('true_wheel_diameter', 'true_wheel_separation'), que puede diferir
    de la nominal configurada en mr_base: asi se reproduce el error sistematico que corrige la calibracion.
  * Deslizamiento aleatorio opcional ('slip_std', fraccion de la velocidad de cada rueda).
Publica /sim/ground_truth (nav_msgs/Odometry, marco 'world') para el simulador de lidar y para evaluar errores.
"""
import math
import os
import pty
import random
import struct
import threading
import time
import tty

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

from mr_base.roboclaw import Cmd, crc16

WRITE_LEN = {Cmd.M1FORWARD: 1, Cmd.M2FORWARD: 1, Cmd.RESETENC: 0, Cmd.MIXEDDUTY: 4,
             Cmd.MIXEDSPEED: 8, Cmd.MIXEDSPEEDACCEL: 12}
READ_CMDS = {Cmd.GETM1ENC, Cmd.GETM2ENC, Cmd.GETM1SPEED, Cmd.GETM2SPEED, Cmd.GETVERSION, Cmd.GETMBATT,
             Cmd.GETLBATT, Cmd.GETCURRENTS, Cmd.READM1PID, Cmd.READM2PID, Cmd.GETTEMP, Cmd.GETERROR}


class Motor:
    def __init__(self, qpps_max, tau):
        self.qpps_max, self.tau = qpps_max, tau
        self.target = 0.0      # QPPS deseado
        self.ramp = 0.0        # consigna con rampa
        self.accel = 0.0       # 0 => sin rampa
        self.speed = 0.0       # QPPS real
        self.count = 0.0
        self.duty_mode = False

    def step(self, dt):
        if self.accel > 0:
            d = self.target - self.ramp
            lim = self.accel * dt
            self.ramp += max(-lim, min(lim, d))
        else:
            self.ramp = self.target
        self.ramp = max(-self.qpps_max, min(self.qpps_max, self.ramp))
        self.speed += (self.ramp - self.speed) * min(1.0, dt / self.tau)
        self.count += self.speed * dt


class Emulator(Node):
    def __init__(self):
        super().__init__('roboclaw_emulator')
        p = self.declare_parameter
        self.link = p('link', '/tmp/roboclaw_emu').value
        self.address = int(p('address', 128).value)
        self.cpr = float(p('encoder_cpr', 48.0).value)
        self.gear = float(p('gear_ratio', 46.85).value)
        self.d_true = float(p('true_wheel_diameter', 0.0875).value)     # rueda real algo gastada/comprimida
        self.b_true = float(p('true_wheel_separation', 0.2090).value)   # contacto efectivo real
        self.left_motor = int(p('left_motor', 2).value)
        self.slip = float(p('slip_std', 0.0).value)
        self.x, self.y, self.th = (float(p('x0', 1.0).value), float(p('y0', 1.0).value),
                                   float(p('yaw0', 0.0).value))
        qpps_max = float(p('qpps_max', 6000.0).value)
        tau = float(p('motor_tau', 0.06).value)
        self.m = {1: Motor(qpps_max, tau), 2: Motor(qpps_max, tau)}
        self.th_unwrapped = self.th
        self.lock = threading.Lock()
        self.stats = {'rx_ok': 0, 'crc_err': 0}

        self.master, slave = pty.openpty()
        tty.setraw(self.master)
        tty.setraw(slave)
        self.slave_name = os.ttyname(slave)
        if os.path.islink(self.link) or os.path.exists(self.link):
            os.remove(self.link)
        os.symlink(self.slave_name, self.link)
        self.get_logger().info(f'RoboClaw emulado en {self.link} -> {self.slave_name} '
                               f'(D_real={self.d_true} m, b_real={self.b_true} m)')
        self.gt_pub = self.create_publisher(Odometry, '/sim/ground_truth', 10)
        threading.Thread(target=self.serial_loop, daemon=True).start()
        threading.Thread(target=self.physics_loop, daemon=True).start()
        self.create_timer(1.0 / 30.0, self.publish_truth)

    # ---------------------------------------------------------------- fisica
    def physics_loop(self):
        dt = 0.01
        k = math.pi * self.d_true / (self.cpr * self.gear)   # metros por cuenta (geometria REAL)
        nxt = time.monotonic()
        while rclpy.ok():
            with self.lock:
                for mot in self.m.values():
                    mot.step(dt)
                ql = self.m[self.left_motor].speed
                qr = self.m[3 - self.left_motor].speed
                vl, vr = ql * k, qr * k
                if self.slip > 0:
                    vl *= 1.0 + random.gauss(0, self.slip)
                    vr *= 1.0 + random.gauss(0, self.slip)
                v, w = 0.5 * (vl + vr), (vr - vl) / self.b_true
                self.x += v * math.cos(self.th + 0.5 * w * dt) * dt
                self.y += v * math.sin(self.th + 0.5 * w * dt) * dt
                self.th = math.atan2(math.sin(self.th + w * dt), math.cos(self.th + w * dt))
                self.th_unwrapped += w * dt
                self.v, self.w = v, w
            nxt += dt
            time.sleep(max(0.0, nxt - time.monotonic()))

    def publish_truth(self):
        with self.lock:
            x, y, th, v, w = self.x, self.y, self.th, getattr(self, 'v', 0.0), getattr(self, 'w', 0.0)
        m = Odometry()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id, m.child_frame_id = 'world', 'base_footprint_truth'
        m.pose.pose.position.x, m.pose.pose.position.y = x, y
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(th / 2), math.cos(th / 2)
        m.twist.twist.linear.x, m.twist.twist.angular.z = v, w
        self.gt_pub.publish(m)

    # ---------------------------------------------------------------- protocolo
    def read_exact(self, n, timeout=0.2):
        buf = b''
        t0 = time.monotonic()
        while len(buf) < n and time.monotonic() - t0 < timeout:
            try:
                buf += os.read(self.master, n - len(buf))
            except BlockingIOError:
                time.sleep(0.001)
        return buf

    def reply(self, head, body):
        os.write(self.master, body + struct.pack('>H', crc16(head + body)))

    def serial_loop(self):
        while rclpy.ok():
            b = os.read(self.master, 1)
            if not b or b[0] != self.address:
                continue
            c = self.read_exact(1)
            if not c:
                continue
            cmd = c[0]
            head = b + c
            if cmd in WRITE_LEN:
                rest = self.read_exact(WRITE_LEN[cmd] + 2)
                if len(rest) != WRITE_LEN[cmd] + 2:
                    continue
                payload, rx = rest[:-2], struct.unpack('>H', rest[-2:])[0]
                if crc16(head + payload) != rx:
                    self.stats['crc_err'] += 1
                    continue
                self.do_write(cmd, payload)
                os.write(self.master, b'\xff')
                self.stats['rx_ok'] += 1
            elif cmd in READ_CMDS:
                self.reply(head, self.do_read(cmd))
                self.stats['rx_ok'] += 1

    def do_write(self, cmd, pl):
        with self.lock:
            if cmd == Cmd.RESETENC:
                self.m[1].count = self.m[2].count = 0.0
            elif cmd == Cmd.MIXEDSPEED:
                s1, s2 = struct.unpack('>ii', pl)
                for i, s in ((1, s1), (2, s2)):
                    self.m[i].target, self.m[i].accel, self.m[i].duty_mode = float(s), 0.0, False
            elif cmd == Cmd.MIXEDSPEEDACCEL:
                a, s1, s2 = struct.unpack('>Iii', pl)
                for i, s in ((1, s1), (2, s2)):
                    self.m[i].target, self.m[i].accel, self.m[i].duty_mode = float(s), float(a), False
            elif cmd == Cmd.MIXEDDUTY:
                d1, d2 = struct.unpack('>hh', pl)
                for i, d in ((1, d1), (2, d2)):
                    mot = self.m[i]
                    mot.target, mot.accel, mot.duty_mode = d / 32767.0 * mot.qpps_max, 0.0, True
            elif cmd in (Cmd.M1FORWARD, Cmd.M2FORWARD):
                i = 1 if cmd == Cmd.M1FORWARD else 2
                self.m[i].target, self.m[i].accel = pl[0] / 127.0 * self.m[i].qpps_max, 0.0

    def do_read(self, cmd):
        with self.lock:
            if cmd in (Cmd.GETM1ENC, Cmd.GETM2ENC):
                mot = self.m[1 if cmd == Cmd.GETM1ENC else 2]
                c = int(math.floor(mot.count)) & 0xFFFFFFFF
                return struct.pack('>IB', c, 0x02 if mot.speed < 0 else 0x00)
            if cmd in (Cmd.GETM1SPEED, Cmd.GETM2SPEED):
                mot = self.m[1 if cmd == Cmd.GETM1SPEED else 2]
                return struct.pack('>IB', int(abs(mot.speed)), 1 if mot.speed < 0 else 0)
            if cmd == Cmd.GETVERSION:
                return b'USB Roboclaw 2x15a v4.2.8 (EMULADOR)\n\x00'
            if cmd == Cmd.GETMBATT:
                return struct.pack('>H', 121)
            if cmd == Cmd.GETLBATT:
                return struct.pack('>H', 50)
            if cmd == Cmd.GETTEMP:
                return struct.pack('>H', 312)
            if cmd == Cmd.GETCURRENTS:
                c1, c2 = (int(abs(self.m[i].speed) / self.m[i].qpps_max * 120) for i in (1, 2))
                return struct.pack('>hh', c1, c2)
            if cmd == Cmd.GETERROR:
                return struct.pack('>I', 0)
            if cmd in (Cmd.READM1PID, Cmd.READM2PID):
                return struct.pack('>IIII', int(0.6 * 65536), int(0.25 * 65536), 0,
                                   int(self.m[1].qpps_max))
        return b''

    def destroy_node(self):
        try:
            os.remove(self.link)
        except OSError:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    n = Emulator()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
