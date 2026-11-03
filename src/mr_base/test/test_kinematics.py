import math

import pytest

from mr_base.kinematics import (DiffDriveOdometry, corrected_ticks_per_meter, corrected_wheel_separation,
                                limit_twist, ticks_per_meter, twist_to_wheels)
from mr_base.roboclaw import crc16, decode_status, to_s32


def test_ticks_per_meter_datasheet():
    # Pololu 25D 47:1 (46.85:1) con encoder de 48 CPR -> 2248.8 cuentas por vuelta de la salida
    assert 48 * 46.85 == pytest.approx(2248.8, abs=0.1)
    assert ticks_per_meter(48, 46.85, 0.090) == pytest.approx(7953.4, abs=0.5)
    # Pololu 37D 131.25:1 con 64 CPR -> 8400 cuentas por vuelta
    assert ticks_per_meter(64, 131.25, 0.090) == pytest.approx(8400 / (math.pi * 0.09), rel=1e-9)


def test_straight_and_arc():
    tpm, b = 8000.0, 0.2
    od = DiffDriveOdometry(tpm, tpm, b)
    od.update(0, 0, 0.0)
    for k in range(1, 101):                     # 1 m recto en 100 pasos
        od.update(k * 80, k * 80, k * 0.01)
    assert od.pose.x == pytest.approx(1.0, abs=1e-9) and od.pose.y == pytest.approx(0.0, abs=1e-12)
    od.reset()
    od.update(0, 0, 0.0)
    # media vuelta antihoraria en el sitio: dR - dL = pi*b
    n = 200
    arc = math.pi * b / 2
    for k in range(1, n + 1):
        od.update(int(round(-k * arc / n * tpm)), int(round(k * arc / n * tpm)), k * 0.01)
    assert abs(od.pose.theta) == pytest.approx(math.pi, abs=2e-3)
    assert math.hypot(od.pose.x, od.pose.y) < 1e-3


def test_circle_closes():
    """Circulo de radio 0.5 m: la integracion exacta por arcos debe cerrar en el origen."""
    tpm, b, R = 10000.0, 0.2, 0.5
    od = DiffDriveOdometry(tpm, tpm, b)
    L = 2 * math.pi * R
    n = 1000
    sl, sr = L * (R - b / 2) / R, L * (R + b / 2) / R
    od.update(0, 0, 0.0)
    for k in range(1, n + 1):
        od.update(round(sl * k / n * tpm), round(sr * k / n * tpm), k * 0.01)
    assert math.hypot(od.pose.x, od.pose.y) < 2e-3
    assert od.pose.y == pytest.approx(0.0, abs=2e-3)


def test_y_sign_left_turn():
    """Giro a la izquierda (rueda derecha mas rapida) => y positiva (error de signo de la version ROS 1)."""
    od = DiffDriveOdometry(1000, 1000, 0.2)
    od.update(0, 0, 0)
    for k in range(1, 50):
        od.update(k * 8, k * 12, k * 0.02)
    assert od.pose.y > 0 and od.pose.theta > 0


def test_jump_rejected():
    od = DiffDriveOdometry(1000, 1000, 0.2, max_jump_ticks=500)
    od.update(0, 0, 0)
    assert od.update(10, 10, 0.05)
    assert not od.update(100000, 10, 0.1)
    assert od.pose.x == pytest.approx(0.01)


def test_calibration_formulas():
    # odometria 1.000 m, cinta 0.972 m -> mas pulsos por metro
    assert corrected_ticks_per_meter(7953.4, 1.0, 0.972) == pytest.approx(8182.5, abs=0.1)
    # odometria 360 grados, real 347.5 -> separacion efectiva mayor
    assert corrected_wheel_separation(0.2016, 2 * math.pi, math.radians(347.5)) == pytest.approx(0.20885, abs=1e-4)


def test_limits():
    v, w = limit_twist(1.0, 3.0, 0.4, 2.0, 0.2, 0.5)
    vl, vr = twist_to_wheels(v, w, 0.2)
    assert max(abs(vl), abs(vr)) <= 0.5 + 1e-9
    assert w / v == pytest.approx(2.0 / 0.4)          # se conserva la curvatura


def test_protocol_helpers():
    assert crc16(b'123456789') == 0x31C3               # valor de verificacion CRC-16/XMODEM
    assert to_s32(0xFFFFFFFF) == -1
    errs, warns = decode_status(0x0001 | (0x0008 << 16), 'actual')
    assert errs == ['Paro de emergencia (E-Stop)'] and warns == ['Bateria principal baja']
    errs, warns = decode_status(0x0004 | 0x0800)                 # manual rev. 5.6 (RoboClaw V5)
    assert errs == ['E-Stop'] and warns == ['Bateria principal baja (aviso)']
