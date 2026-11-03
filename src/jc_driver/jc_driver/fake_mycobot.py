"""Simulador minimo de la API pymycobot.MyCobot280 (solo para pruebas sin hardware).

Integra los angulos hacia el objetivo con una velocidad proporcional al parametro
'speed' (1-100), de forma similar al robot real. NO modela dinamica ni colisiones.
"""
import threading
import time


class FakeMyCobot280:
    MAX_DEG_S = 150.0  # velocidad articular aproximada a speed=100 (supuesto didactico)

    def __init__(self, port="fake", baud=1000000):
        self._lock = threading.Lock()
        self._angles = [0.0] * 6
        self._target = [0.0] * 6
        self._speed = 50
        self._gripper = 100
        self._gripper_target = 100
        self._t = time.monotonic()
        self._powered = True
        self._fresh = 0

    def _update(self):
        now = time.monotonic()
        dt, self._t = now - self._t, now
        step = self.MAX_DEG_S * self._speed / 100.0 * dt
        for i in range(6):
            e = self._target[i] - self._angles[i]
            self._angles[i] += max(-step, min(step, e))
        g = self._gripper_target - self._gripper
        self._gripper += max(-200 * dt, min(200 * dt, g))

    # --- API compatible con pymycobot (subconjunto) ---
    def get_angles(self):
        with self._lock:
            self._update()
            return [round(a, 2) for a in self._angles]

    def send_angles(self, angles, speed):
        with self._lock:
            self._update()
            self._target = [float(a) for a in angles]
            self._speed = max(1, min(100, int(speed)))

    def get_gripper_value(self):
        with self._lock:
            self._update()
            return int(round(self._gripper))

    def set_gripper_value(self, value, speed, gripper_type=None):
        with self._lock:
            self._update()
            self._gripper_target = max(0, min(100, int(value)))

    def is_moving(self):
        with self._lock:
            self._update()
            return int(any(abs(t - a) > 0.2 for t, a in zip(self._target, self._angles)))

    def is_gripper_moving(self):
        with self._lock:
            self._update()
            return int(abs(self._gripper_target - self._gripper) > 0.5)

    def stop(self):
        with self._lock:
            self._update()
            self._target = list(self._angles)
        return 1

    def power_on(self):
        self._powered = True

    def power_off(self):
        self._powered = False

    def is_power_on(self):
        return int(self._powered)

    def release_all_servos(self, *a):
        return 1

    def focus_all_servos(self):
        return 1

    def get_fresh_mode(self):
        return self._fresh

    def set_fresh_mode(self, mode):
        self._fresh = int(mode)

    def close(self):
        pass
