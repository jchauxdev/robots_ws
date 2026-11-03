"""Cinematica directa (FK) e inversa numerica (IK) a partir del URDF - Día 2 (DA-05).

Implementacion didactica con matrices homogeneas (producto de transformaciones
del URDF) y Jacobiano geometrico; IK por minimos cuadrados amortiguados (DLS):
    dq = J^T (J J^T + lambda^2 I)^-1 e
Ref.: Lynch & Park (2017) Modern Robotics, cap. 4-6; Siciliano et al. (2009) cap. 2-3.
"""
import math

import numpy as np


def _rpy_to_R(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _axis_angle(axis, q):
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(q) * K + (1 - math.cos(q)) * K @ K


class SerialChain:
    """Cadena serie extraida de un URDF entre 'base' y 'tip'."""

    def __init__(self, urdf_xml, base="world", tip="tcp"):
        from urdf_parser_py.urdf import URDF
        robot = URDF.from_xml_string(urdf_xml)
        self.joints = []  # (T_origen 4x4, eje | None, nombre, limites)
        for name in robot.get_chain(base, tip, links=False):
            j = robot.joint_map[name]
            T = np.eye(4)
            if j.origin is not None:
                T[:3, :3] = _rpy_to_R(*j.origin.rpy)
                T[:3, 3] = j.origin.xyz
            mov = j.type in ("revolute", "continuous")
            lim = (j.limit.lower, j.limit.upper) if (mov and j.limit) else (-math.pi, math.pi)
            self.joints.append((T, np.array(j.axis) if mov else None, name, lim))
        self.names = [n for _, a, n, _ in self.joints if a is not None]
        self.limits = np.array([lim for _, a, _, lim in self.joints if a is not None])

    def fk(self, q, return_frames=False):
        T, i, frames = np.eye(4), 0, []
        for T0, axis, _, _ in self.joints:
            T = T @ T0
            if axis is not None:
                frames.append((T[:3, 3].copy(), T[:3, :3] @ axis))
                R = np.eye(4)
                R[:3, :3] = _axis_angle(axis, q[i])
                T = T @ R
                i += 1
        return (T, frames) if return_frames else T

    def jacobian(self, q):
        """Jacobiano geometrico 6xn (velocidad lineal; velocidad angular) en el marco base."""
        T, frames = self.fk(q, True)
        p = T[:3, 3]
        J = np.zeros((6, len(frames)))
        for k, (o, z) in enumerate(frames):
            J[:3, k] = np.cross(z, p - o)
            J[3:, k] = z
        return J

    def ik(self, T_goal, q0, iters=300, lam=0.05, tol=1e-4, w_rot=0.3):
        """IK numerica DLS. Devuelve (q, exito, error_pos_m)."""
        q = np.array(q0, float)
        W = np.diag([1, 1, 1, w_rot, w_rot, w_rot])
        for _ in range(iters):
            T = self.fk(q)
            e_p = T_goal[:3, 3] - T[:3, 3]
            Re = T_goal[:3, :3] @ T[:3, :3].T
            ang = math.acos(max(-1.0, min(1.0, (np.trace(Re) - 1) / 2)))
            if ang < 1e-9:
                e_o = np.zeros(3)
            else:
                e_o = ang / (2 * math.sin(ang)) * np.array([Re[2, 1] - Re[1, 2], Re[0, 2] - Re[2, 0], Re[1, 0] - Re[0, 1]])
            e = W @ np.concatenate([e_p, e_o])
            if np.linalg.norm(e_p) < tol and np.linalg.norm(e_o) < 10 * tol:
                return q, True, float(np.linalg.norm(e_p))
            J = W @ self.jacobian(q)
            dq = J.T @ np.linalg.solve(J @ J.T + lam ** 2 * np.eye(6), e)
            q = np.clip(q + dq, self.limits[:, 0], self.limits[:, 1])
        return q, False, float(np.linalg.norm(T_goal[:3, 3] - self.fk(q)[:3, 3]))

    def manipulability(self, q):
        """Indice de Yoshikawa w = sqrt(det(J J^T)); w -> 0 cerca de singularidades."""
        J = self.jacobian(q)
        return math.sqrt(max(0.0, np.linalg.det(J @ J.T)))
