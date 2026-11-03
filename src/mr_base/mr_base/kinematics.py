"""Cinematica y odometria del robot diferencial (sin dependencias de ROS, probada con pytest).

Convenciones (REP 103): x adelante, y izquierda, theta antihorario positivo.
  v     = (v_R + v_L) / 2
  omega = (v_R - v_L) / b          (b = separacion entre ruedas, wheel_separation)
  v_R   = v + omega*b/2,  v_L = v - omega*b/2

Integracion exacta por arco de circunferencia entre dos lecturas de encoder (Siegwart et al., 2011, cap. 5):
  ds = (dR + dL)/2,  dth = (dR - dL)/b
  si |dth| ~ 0:  x += ds cos(th), y += ds sin(th)
  si no:         R = ds/dth
                 x += R [sin(th+dth) - sin(th)]
                 y -= R [cos(th+dth) - cos(th)]
                 th += dth
"""
from __future__ import annotations

import math
from dataclasses import dataclass


def ticks_per_meter(encoder_cpr: float, gear_ratio: float, wheel_diameter: float) -> float:
    """Pulsos de cuadratura por metro recorrido por la rueda.

    encoder_cpr: cuentas por vuelta del EJE DEL MOTOR (Pololu 25D: 48; 37D: 64; contando 4 flancos).
    gear_ratio : relacion exacta de la caja (p. ej. 46.85, 131.25).
    """
    return encoder_cpr * gear_ratio / (math.pi * wheel_diameter)


def normalize_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def twist_to_wheels(v: float, w: float, wheel_separation: float) -> tuple[float, float]:
    """(v_L, v_R) en m/s."""
    return v - w * wheel_separation / 2.0, v + w * wheel_separation / 2.0


def limit_twist(v: float, w: float, v_max: float, w_max: float, wheel_separation: float,
                wheel_v_max: float | None = None) -> tuple[float, float]:
    """Satura v y w y, si se indica, escala ambos para no superar la velocidad maxima de rueda
    (conserva la curvatura w/v, a diferencia de saturar cada rueda por separado)."""
    v = max(-v_max, min(v_max, v))
    w = max(-w_max, min(w_max, w))
    if wheel_v_max:
        vl, vr = twist_to_wheels(v, w, wheel_separation)
        m = max(abs(vl), abs(vr))
        if m > wheel_v_max:
            k = wheel_v_max / m
            v, w = v * k, w * k
    return v, w


@dataclass
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0


class DiffDriveOdometry:
    def __init__(self, ticks_per_meter_left: float, ticks_per_meter_right: float, wheel_separation: float,
                 max_jump_ticks: int = 100000):
        self.tpm_l, self.tpm_r = ticks_per_meter_left, ticks_per_meter_right
        self.b = wheel_separation
        self.max_jump = max_jump_ticks
        self.pose = Pose2D()
        self.v = 0.0
        self.w = 0.0
        self.dist_left = 0.0     # distancia acumulada de cada rueda (m), util para calibracion
        self.dist_right = 0.0
        self._last = None
        self.rejected = 0

    def reset(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0):
        self.pose = Pose2D(x, y, theta)
        self.dist_left = self.dist_right = 0.0
        self._last = None

    def update(self, ticks_left: int, ticks_right: int, t: float) -> bool:
        """Actualiza con las cuentas absolutas de encoder y el instante t [s]. False si se descarta."""
        if self._last is None:
            self._last = (ticks_left, ticks_right, t)
            return True
        l0, r0, t0 = self._last
        dl_t, dr_t = ticks_left - l0, ticks_right - r0
        dt = t - t0
        if abs(dl_t) > self.max_jump or abs(dr_t) > self.max_jump:
            # salto imposible (reinicio del contador, error de comunicacion): se re-sincroniza
            self._last = (ticks_left, ticks_right, t)
            self.rejected += 1
            return False
        self._last = (ticks_left, ticks_right, t)
        dl, dr = dl_t / self.tpm_l, dr_t / self.tpm_r
        self.dist_left += dl
        self.dist_right += dr
        ds = 0.5 * (dr + dl)
        dth = (dr - dl) / self.b
        th = self.pose.theta
        if abs(dth) < 1e-9:
            self.pose.x += ds * math.cos(th)
            self.pose.y += ds * math.sin(th)
        else:
            r = ds / dth
            self.pose.x += r * (math.sin(th + dth) - math.sin(th))
            self.pose.y -= r * (math.cos(th + dth) - math.cos(th))
        self.pose.theta = normalize_angle(th + dth)
        if dt > 1e-6:
            self.v, self.w = ds / dt, dth / dt
        return True


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    """(x, y, z, w) de una rotacion pura en z."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


# --------------------------------------------------------------------- calibracion
def corrected_ticks_per_meter(tpm_actual: float, distancia_odometria: float, distancia_real: float) -> float:
    """Si la odometria reporto d_odo pero el robot recorrio d_real (cinta metrica):
    tpm_nuevo = tpm_actual * d_odo / d_real   (mas pulsos por metro si el robot recorrio menos)."""
    if distancia_real <= 0:
        raise ValueError("distancia_real debe ser positiva")
    return tpm_actual * distancia_odometria / distancia_real


def corrected_wheel_separation(b_actual: float, angulo_odometria: float, angulo_real: float) -> float:
    """Si la odometria reporto un giro theta_odo pero el robot giro theta_real:
    b_nuevo = b_actual * theta_odo / theta_real."""
    if angulo_real == 0:
        raise ValueError("angulo_real no puede ser 0")
    return b_actual * angulo_odometria / angulo_real
