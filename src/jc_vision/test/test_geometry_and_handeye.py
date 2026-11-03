"""Pruebas unitarias (pytest): geometria de proyeccion y calibracion mano-ojo con datos sinteticos."""
import math

import cv2
import numpy as np

from jc_vision import geometry_utils as gu
from jc_vision.vision_algorithms import (classify_shape, find_blobs, hand_eye_consistency,
                                         hsv_mask, solve_hand_eye)


def _rand_T(rng, rot_deg=30, trans=0.05):
    rvec = np.radians(rng.uniform(-rot_deg, rot_deg, 3))
    return gu.make_T(cv2.Rodrigues(rvec)[0], rng.uniform(-trans, trans, 3))


def test_quat_roundtrip():
    q = gu.quat_from_rpy(math.pi, 0.0, -math.pi / 2)
    q2 = gu.matrix_to_quat(gu.quat_to_matrix(q))
    assert min(np.linalg.norm(q - q2), np.linalg.norm(q + q2)) < 1e-9


def test_pixel_to_plane_center():
    K = np.array([[554.2563, 0, 320], [0, 554.2563, 240], [0, 0, 1]])
    # camara 0.30 m sobre la mesa mirando hacia abajo (z optico = -z base)
    R = np.diag([1.0, -1.0, -1.0])
    T = gu.make_T(R, [0.15, -0.05, 0.30])
    p = gu.pixel_to_base(320, 240, K, None, T, 0.0)
    assert np.allclose(p, [0.15, -0.05, 0.0], atol=1e-9)
    # 100 px a la derecha => 0.30*100/554.26 = 0.0541 m en x
    p = gu.pixel_to_base(420, 240, K, None, T, 0.0)
    assert abs(p[0] - (0.15 + 0.30 * 100 / 554.2563)) < 1e-6


def test_hand_eye_recovers_ground_truth():
    rng = np.random.default_rng(0)
    X = gu.make_T(cv2.Rodrigues(np.array([0.05, -1.5, 0.02]))[0], [0.036, 0.0, 0.02])  # brida <- camara
    T_base_target = gu.make_T(np.diag([1.0, -1.0, -1.0]), [0.18, -0.05, 0.0])
    A, B = [], []
    for _ in range(15):
        Tbg = gu.make_T(cv2.Rodrigues(np.radians(rng.uniform(-25, 25, 3)) + np.array([math.pi, 0, 0]))[0],
                        [0.15, -0.05, 0.30] + rng.uniform(-0.05, 0.05, 3))
        A.append(Tbg)
        B.append(gu.invert_T(X) @ gu.invert_T(Tbg) @ T_base_target)
    for method in ["TSAI", "PARK", "HORAUD", "DANIILIDIS"]:
        Xe = solve_hand_eye(A, B, method)
        assert np.linalg.norm(Xe[:3, 3] - X[:3, 3]) < 1e-3, method
        _, mean_e, _ = hand_eye_consistency(A, B, Xe)
        assert mean_e < 1e-3


def test_color_and_shape():
    img = np.full((240, 320, 3), 200, np.uint8)
    cv2.rectangle(img, (40, 40), (100, 100), (0, 0, 220), -1)      # cuadrado rojo (BGR)
    cv2.circle(img, (220, 120), 35, (220, 30, 0), -1)               # circulo azul
    red = find_blobs(hsv_mask(img, [[(0, 120, 70), (10, 255, 255)], [(170, 120, 70), (179, 255, 255)]]))
    blue = find_blobs(hsv_mask(img, [[(94, 80, 40), (126, 255, 255)]]))
    assert len(red) == 1 and red[0]["shape"] == "cuadrado"
    assert abs(red[0]["u"] - 70) < 2 and abs(red[0]["v"] - 70) < 2
    assert len(blue) == 1 and blue[0]["shape"] == "circulo"
    tri = np.array([[10, 10], [60, 10], [35, 60]], np.int32).reshape(-1, 1, 2)
    assert classify_shape(tri) == "triangulo"
