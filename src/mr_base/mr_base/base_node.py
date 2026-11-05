"""Nodo base del robot movil: RoboClaw 2x15A <-> ROS 2 Humble.

Suscribe:  cmd_vel (geometry_msgs/Twist)
Publica:   odom (nav_msgs/Odometry), TF odom->base_footprint, joint_states (ruedas),
           diagnostics (bateria, temperatura, corrientes, estado), ~/wheel_ticks (para calibracion)
Servicios: ~/reset_odometry (std_srvs/Trigger)

Reemplaza a roboclaw_node (ROS 1) del curso anterior y corrige sus fallas conocidas:
  - mapeo de motores explicito (left_motor) e inversion por rueda (en ROS 1 M1 recibia la rueda derecha,
    pero el encoder 1 se integraba como izquierdo: la y y el giro quedaban invertidos);
  - TF y mensaje de odometria con el MISMO angulo (en ROS 1 el TF usaba -theta);
  - perro guardian (cmd_timeout) con rampa de frenado y parada en ciclo util 0;
  - ticks_per_meter por rueda y calculo a partir de la hoja de datos si no se fija.
"""
import threading

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Int64MultiArray
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from .kinematics import DiffDriveOdometry, limit_twist, ticks_per_meter, twist_to_wheels, yaw_to_quaternion
from .roboclaw import RoboClaw, decode_status


class BaseNode(Node):
    def __init__(self):
        super().__init__('mr_base')
        p = self.declare_parameter
        self.port = p('port', '/dev/roboclaw').value
        self.baud = int(p('baud', 38400).value)
        self.address = int(p('address', 128).value)
        # Geometria y encoders
        self.b = float(p('wheel_separation', 0.2016).value)
        diam = float(p('wheel_diameter', 0.090).value)
        cpr = float(p('encoder_cpr', 48.0).value)
        gear = float(p('gear_ratio', 46.85).value)
        tpm = float(p('ticks_per_meter', 0.0).value)          # 0 => se calcula de la hoja de datos
        self.tpm = tpm if tpm > 0 else ticks_per_meter(cpr, gear, diam)
        self.tpm_l = self.tpm * float(p('left_scale', 1.0).value)    # correccion fina por rueda (UMBmark)
        self.tpm_r = self.tpm * float(p('right_scale', 1.0).value)
        self.wheel_radius = diam / 2.0
        self.left_motor = int(p('left_motor', 2).value)       # cableado: 1 = M1, 2 = M2
        self.inv_l = bool(p('invert_left', False).value)
        self.inv_r = bool(p('invert_right', False).value)
        # Limites y control
        self.v_max = float(p('max_linear_speed', 0.4).value)
        self.w_max = float(p('max_angular_speed', 2.0).value)
        self.wheel_v_max = float(p('max_wheel_speed', 0.5).value)
        self.accel = float(p('acceleration', 0.6).value)      # m/s^2 en cada rueda
        self.cmd_timeout = float(p('cmd_timeout', 0.5).value)
        rate = float(p('rate', 20.0).value)
        self.status_gen = p('status_bits', 'v5').value      # 'v5' (manual rev. 5.6) | 'actual'
        # Marcos
        self.odom_frame = p('odom_frame', 'odom').value
        self.base_frame = p('base_frame', 'base_footprint').value
        self.publish_tf = bool(p('publish_tf', True).value)
        self.left_joint = p('left_joint', 'left_wheel_joint').value
        self.right_joint = p('right_joint', 'right_wheel_joint').value

        self.rc = RoboClaw(self.port, self.baud, self.address)
        try:
            self.rc.open()
        except Exception as e:  # noqa: BLE001
            self.get_logger().fatal(f'No se pudo abrir {self.port}: {e}')
            raise
        version = self.rc.read_version()
        self.get_logger().info(f'RoboClaw: {version or "sin respuesta a ReadVersion"} en {self.port}')
        for m in (1, 2):
            pid = self.rc.read_velocity_pid(m)
            if pid:
                self.get_logger().info(f'PID velocidad M{m}: P={pid["p"]:.3f} I={pid["i"]:.3f} '
                                       f'D={pid["d"]:.3f} QPPS={pid["qpps"]}')
                if pid['qpps'] == 0:
                    self.get_logger().warn(f'M{m} sin QPPS: ejecute Autotune en Motion Studio (DA-10.1)')
        self.rc.stop()
        self.rc.reset_encoders()
        self.get_logger().info(f'ticks_per_meter={self.tpm:.1f} (izq {self.tpm_l:.1f}, der {self.tpm_r:.1f}), '
                               f'wheel_separation={self.b:.4f} m, motor izquierdo=M{self.left_motor}')

        self.odom = DiffDriveOdometry(self.tpm_l, self.tpm_r, self.b,
                                      max_jump_ticks=int(self.tpm * 2.0 / rate + 2000))
        self.cmd = (0.0, 0.0)
        self.last_cmd_time = None
        self.stopped = True
        self.lock = threading.Lock()
        self.link_up = True

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.js_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, 'diagnostics', 10)
        self.ticks_pub = self.create_publisher(Int64MultiArray, '~/wheel_ticks', 10)
        self.tf = TransformBroadcaster(self) if self.publish_tf else None
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd, qos_profile_sensor_data)
        self.create_service(Trigger, '~/reset_odometry', self.on_reset)
        self.create_timer(1.0 / rate, self.loop)
        self.create_timer(1.0, self.diagnostics)

    # ------------------------------------------------------------------ utilidades
    def motor_of(self, side):
        return self.left_motor if side == 'left' else 3 - self.left_motor

    def read_ticks(self):
        el = self.rc.read_encoder(self.motor_of('left'))
        er = self.rc.read_encoder(self.motor_of('right'))
        if el is None or er is None:
            return None
        tl, tr = el[0], er[0]
        return (-tl if self.inv_l else tl), (-tr if self.inv_r else tr)

    def send_wheels(self, vl, vr):
        ql, qr = int(round(vl * self.tpm_l)), int(round(vr * self.tpm_r))
        ql, qr = (-ql if self.inv_l else ql), (-qr if self.inv_r else qr)
        q = {self.motor_of('left'): ql, self.motor_of('right'): qr}
        accel = int(self.accel * self.tpm)
        return self.rc.speed_accel_m1m2(accel, q[1], q[2])

    # ------------------------------------------------------------------ callbacks
    def on_cmd(self, msg: Twist):
        v, w = limit_twist(msg.linear.x, msg.angular.z, self.v_max, self.w_max, self.b, self.wheel_v_max)
        with self.lock:
            self.cmd = (v, w)
            self.last_cmd_time = self.get_clock().now()

    def on_reset(self, _req, res):
        with self.lock:
            self.rc.reset_encoders()
            self.odom.reset()
        res.success, res.message = True, 'Odometria y encoders en cero'
        return res

    def loop(self):
        now = self.get_clock().now()
        with self.lock:
            v, w = self.cmd
            stale = self.last_cmd_time is None or \
                (now - self.last_cmd_time).nanoseconds * 1e-9 > self.cmd_timeout
        if stale:
            v = w = 0.0
        if v == 0.0 and w == 0.0:
            if not self.stopped:
                # rampa a cero con el PID y, ya detenido, ciclo util 0 (evita zumbido del PID en reposo)
                self.send_wheels(0.0, 0.0)
                if abs(self.odom.v) < 0.01 and abs(self.odom.w) < 0.02:
                    self.rc.stop()
                    self.stopped = True
        else:
            self.stopped = False
            self.send_wheels(*twist_to_wheels(v, w, self.b))

        ticks = self.read_ticks()
        self.check_link()
        if ticks is None:
            if self.rc.connected:
                self.get_logger().warn('Lectura de encoders fallida', throttle_duration_sec=2.0)
            return
        t = now.nanoseconds * 1e-9
        if not self.odom.update(ticks[0], ticks[1], t):
            self.get_logger().warn('Salto de encoder descartado', throttle_duration_sec=2.0)
        self.publish(now, ticks)

    def check_link(self):
        """Avisa cuando se pierde o se recupera el puerto del RoboClaw (el nodo sigue vivo y reintenta)."""
        up = self.rc.connected
        if up == self.link_up:
            if not up:
                self.get_logger().warn(f'Sin conexion con el RoboClaw en {self.port}: reintentando',
                                       throttle_duration_sec=5.0)
            return
        self.link_up = up
        if up:
            self.get_logger().info(f'RoboClaw reconectado en {self.port}')
            self.stopped = False
        else:
            self.get_logger().error(
                f'Se perdio la conexion con el RoboClaw en {self.port} ({self.rc.last_error}). '
                'Causas habituales: cable USB flojo o con ruido, caida de tension al arrancar los motores, '
                'u otro programa usando el puerto (otro mr_base, el servicio mr-robot, ModemManager). '
                'Se reintenta cada segundo.')

    def publish(self, now, ticks):
        pose, stamp = self.odom.pose, now.to_msg()
        q = yaw_to_quaternion(pose.theta)
        od = Odometry()
        od.header.stamp, od.header.frame_id, od.child_frame_id = stamp, self.odom_frame, self.base_frame
        od.pose.pose.position.x, od.pose.pose.position.y = pose.x, pose.y
        (od.pose.pose.orientation.x, od.pose.pose.orientation.y,
         od.pose.pose.orientation.z, od.pose.pose.orientation.w) = q
        od.twist.twist.linear.x, od.twist.twist.angular.z = self.odom.v, self.odom.w
        # covarianzas diagonales (x, y, z, rx, ry, rz): z/rx/ry no observables en el plano
        for i, c in zip((0, 7, 14, 21, 28, 35), (1e-3, 1e-3, 1e6, 1e6, 1e6, 3e-3)):
            od.pose.covariance[i] = c
            od.twist.covariance[i] = c
        self.odom_pub.publish(od)
        if self.tf:
            tf = TransformStamped()
            tf.header.stamp, tf.header.frame_id, tf.child_frame_id = stamp, self.odom_frame, self.base_frame
            tf.transform.translation.x, tf.transform.translation.y = pose.x, pose.y
            (tf.transform.rotation.x, tf.transform.rotation.y,
             tf.transform.rotation.z, tf.transform.rotation.w) = q
            self.tf.sendTransform(tf)
        js = JointState()
        js.header.stamp = stamp
        js.name = [self.left_joint, self.right_joint]
        js.position = [ticks[0] / self.tpm_l / self.wheel_radius, ticks[1] / self.tpm_r / self.wheel_radius]
        self.js_pub.publish(js)
        self.ticks_pub.publish(Int64MultiArray(data=[int(ticks[0]), int(ticks[1])]))

    def diagnostics(self):
        st = DiagnosticStatus(name='mr_base: RoboClaw', hardware_id=f'roboclaw@{self.port}')
        vals = {}
        with self.lock:
            mb = self.rc.read_main_battery()
            lb = self.rc.read_logic_battery()
            tmp = self.rc.read_temperature()
            cur = self.rc.read_currents()
            status = self.rc.read_status()
        if mb is not None:
            vals['bateria_principal_V'] = f'{mb:.1f}'
        if lb is not None:
            vals['bateria_logica_V'] = f'{lb:.1f}'
        if tmp is not None:
            vals['temperatura_C'] = f'{tmp:.1f}'
        if cur is not None:
            vals['corriente_M1_A'], vals['corriente_M2_A'] = f'{cur[0]:.2f}', f'{cur[1]:.2f}'
        vals['errores_crc'] = str(self.rc.crc_errors)
        vals['conectado'] = 'si' if self.rc.connected else 'no'
        vals['desconexiones'] = str(self.rc.io_errors)
        vals['reconexiones'] = str(self.rc.reconnects)
        st.level, st.message = DiagnosticStatus.OK, 'OK'
        if status is None:
            st.level, st.message = DiagnosticStatus.WARN, 'Sin respuesta a Read Status'
        else:
            vals['estado_hex'] = f'0x{status:08X}'
            errs, warns = decode_status(status, self.status_gen)
            if errs:
                st.level, st.message = DiagnosticStatus.ERROR, '; '.join(errs)
            elif warns:
                st.level, st.message = DiagnosticStatus.WARN, '; '.join(warns)
        if not self.rc.connected:
            st.level, st.message = DiagnosticStatus.ERROR, f'Sin conexion con el RoboClaw ({self.rc.last_error})'
        st.values = [KeyValue(key=k, value=v) for k, v in vals.items()]
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [st]
        self.diag_pub.publish(arr)

    def destroy_node(self):
        try:
            self.rc.stop()
            self.rc.close()
        except Exception:  # noqa: BLE001
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = BaseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
