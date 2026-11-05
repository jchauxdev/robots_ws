"""Pruebas: FK/IK numericas sobre el URDF generado con xacro."""
import math
import os
import subprocess

import numpy as np
import pytest

from jc_apps.kinematics import SerialChain
from jc_apps.pick_place_node import best_tcp_yaw


def _urdf():
    try:
        from ament_index_python.packages import get_package_share_directory
        x = os.path.join(get_package_share_directory("jc_description"), "urdf", "jetcobot.urdf.xacro")
    except Exception:
        pytest.skip("jc_description no instalado")
    return subprocess.check_output(["xacro", x, "sim_mode:=real"]).decode()


def test_fk_zero_and_ready():
    ch = SerialChain(_urdf(), "world", "tcp")
    T0 = ch.fk(np.zeros(6))
    assert np.allclose(T0[:3, 3], [0.1406, -0.0646, 0.4140], atol=1e-3)
    Tr = ch.fk(np.radians([0, 0, -90, 0, 0, 0]))
    assert np.allclose(Tr[:3, 2], [0, 0, -1], atol=1e-6)  # TCP hacia abajo


def test_ik_roundtrip():
    ch = SerialChain(_urdf(), "world", "tcp")
    q_true = np.radians([20, 10, -100, -10, 5, 15])
    T = ch.fk(q_true)
    q, ok, err = ch.ik(T, np.radians([0, 0, -90, 0, 0, 0]))
    assert ok and err < 1e-3
    assert np.allclose(ch.fk(q)[:3, 3], T[:3, 3], atol=1e-3)


def test_best_yaw():
    assert abs(best_tcp_yaw(0.0) - (-math.pi / 2)) < 1e-9
    assert abs(best_tcp_yaw(math.radians(10)) - math.radians(-80)) < 1e-9
