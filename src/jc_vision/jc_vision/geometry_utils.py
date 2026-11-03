"""Utilidades geometricas puras (sin ROS) para vision 3D y calibracion.

Convenciones:
  * Matrices homogeneas 4x4 'T_a_b' transforman puntos expresados en el marco b al marco a:
        p_a = T_a_b @ p_b
  * Cuaterniones en orden (x, y, z, w), como geometry_msgs/Quaternion.
  * Marco optico de camara (REP-103): z hacia adelante, x a la derecha, y hacia abajo.
"""
import math

import numpy as np


def quat_to_matrix(q):
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def matrix_to_quat(R):
    """Conversion robusta matriz de rotacion -> cuaternion (x, y, z, w)."""
    R = np.asarray(R, dtype=float)
    tr = np.trace(R)
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w, x, y, z = 0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w, x, y, z = (R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w, x, y, z = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w, x, y, z = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


def quat_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return np.array([
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ])


def make_T(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.ravel(t)
    return T


def invert_T(T):
    R, t = T[:3, :3], T[:3, 3]
    return make_T(R.T, -R.T @ t)


def transform_msg_to_T(tf):
    """geometry_msgs/Transform (o TransformStamped.transform) -> matriz 4x4."""
    t, q = tf.translation, tf.rotation
    return make_T(quat_to_matrix((q.x, q.y, q.z, q.w)), (t.x, t.y, t.z))


def K_from_camera_info(info):
    K = np.array(info.k, dtype=float).reshape(3, 3)
    D = np.array(info.d, dtype=float) if len(info.d) else np.zeros(5)
    return K, D


def pixel_to_ray(u, v, K, D=None):
    """Rayo unitario en el marco optico para el pixel (u, v). Corrige distorsion si D != 0."""
    if D is not None and np.any(np.abs(D) > 1e-12):
        import cv2
        pts = cv2.undistortPoints(np.array([[[float(u), float(v)]]]), K, D)
        x, y = pts[0, 0]
    else:
        x = (u - K[0, 2]) / K[0, 0]
        y = (v - K[1, 2]) / K[1, 1]
    d = np.array([x, y, 1.0])
    return d / np.linalg.norm(d)


def intersect_ray_plane(T_base_cam, ray_cam, plane_z):
    """Interseca el rayo (desde el centro optico) con el plano z = plane_z del marco base.

    Devuelve el punto 3D en el marco base o None si el rayo es paralelo o apunta lejos.
    """
    o = T_base_cam[:3, 3]
    d = T_base_cam[:3, :3] @ ray_cam
    if abs(d[2]) < 1e-9:
        return None
    s = (plane_z - o[2]) / d[2]
    if s <= 0:
        return None
    return o + s * d


def pixel_to_base(u, v, K, D, T_base_cam, plane_z):
    return intersect_ray_plane(T_base_cam, pixel_to_ray(u, v, K, D), plane_z)


def yaw_in_base(u, v, angle_rad, K, D, T_base_cam, plane_z, length_px=20.0):
    """Orientacion (yaw) en el plano de la mesa de un eje observado en la imagen."""
    p0 = pixel_to_base(u, v, K, D, T_base_cam, plane_z)
    p1 = pixel_to_base(u + length_px * math.cos(angle_rad), v + length_px * math.sin(angle_rad),
                       K, D, T_base_cam, plane_z)
    if p0 is None or p1 is None:
        return 0.0
    return math.atan2(p1[1] - p0[1], p1[0] - p0[0])


def load_camera_yaml(path):
    """Lee un YAML de calibracion en formato ROS (camera_calibration / camera_info_manager)."""
    import yaml
    with open(path, encoding="utf-8") as f:
        d = yaml.safe_load(f)
    K = np.array(d["camera_matrix"]["data"], dtype=float).reshape(3, 3)
    D = np.array(d["distortion_coefficients"]["data"], dtype=float)
    return d, K, D
