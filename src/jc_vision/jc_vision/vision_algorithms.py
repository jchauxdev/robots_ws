"""Algoritmos de vision clasica (OpenCV) independientes de ROS, reutilizables y testeables."""
import math

import cv2
import numpy as np


def hsv_mask(bgr, ranges, blur=5, open_iter=1, close_iter=2):
    """Mascara binaria por rangos HSV. 'ranges' = [[(h,s,v)min, (h,s,v)max], ...].

    OpenCV usa H en [0,179]; el rojo requiere dos bandas (envuelve 0/179).
    """
    img = cv2.GaussianBlur(bgr, (blur, blur), 0) if blur else bgr
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], np.uint8)
    for lo, hi in ranges:
        mask |= cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    if open_iter:
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k, iterations=open_iter)
    if close_iter:
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=close_iter)
    return mask


def classify_shape(contour):
    """Clasifica un contorno: triangulo, cuadrado, rectangulo, pentagono, hexagono, circulo."""
    peri = cv2.arcLength(contour, True)
    area = cv2.contourArea(contour)
    if peri <= 0 or area <= 0:
        return "desconocido"
    approx = cv2.approxPolyDP(contour, 0.035 * peri, True)
    n = len(approx)
    circularity = 4.0 * math.pi * area / (peri * peri)
    if n == 3:
        return "triangulo"
    if n == 4:
        (_, _), (w, h), _ = cv2.minAreaRect(contour)
        ar = max(w, h) / max(1e-6, min(w, h))
        return "cuadrado" if ar < 1.2 else "rectangulo"
    if circularity > 0.80:
        return "circulo"
    if n == 5:
        return "pentagono"
    if n == 6:
        return "hexagono"
    return "poligono"


def find_blobs(mask, min_area=300, max_area=1e9):
    """Contornos externos con area en rango -> lista de dicts con centroide, caja y angulo."""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    out = []
    for c in contours:
        a = cv2.contourArea(c)
        if a < min_area or a > max_area:
            continue
        m = cv2.moments(c)
        if m["m00"] == 0:
            continue
        cx, cy = m["m10"] / m["m00"], m["m01"] / m["m00"]
        (rx, ry), (w, h), ang = cv2.minAreaRect(c)
        out.append({"contour": c, "area": a, "u": cx, "v": cy, "w": w, "h": h,
                    "angle_rad": math.radians(ang), "shape": classify_shape(c)})
    out.sort(key=lambda b: -b["area"])
    return out


# ---------------------------------------------------------------- ArUco
def aruco_dictionary(name="DICT_4X4_50"):
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))


def detect_aruco(gray, dictionary):
    """Compatible con OpenCV < 4.7 (API antigua) y >= 4.7 (ArucoDetector)."""
    if hasattr(cv2.aruco, "ArucoDetector"):
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        corners, ids, _ = cv2.aruco.ArucoDetector(dictionary, params).detectMarkers(gray)
    else:  # OpenCV 4.5.x (Ubuntu 22.04 python3-opencv)
        params = cv2.aruco.DetectorParameters_create()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=params)
    return corners, ids


def marker_object_points(length):
    """Esquinas del marcador en su propio marco (z = 0), orden de OpenCV: sup-izq, sup-der, inf-der, inf-izq."""
    h = length / 2.0
    return np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float32)


def marker_pose(corners_i, length, K, D):
    """Pose del marcador en el marco optico: (rvec, tvec) con solvePnP IPPE_SQUARE."""
    ok, rvec, tvec = cv2.solvePnP(marker_object_points(length),
                                  corners_i.reshape(4, 2).astype(np.float32),
                                  K, D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    return (rvec, tvec) if ok else (None, None)


# ---------------------------------------------------------------- Hand-eye
HANDEYE_METHODS = {
    "TSAI": cv2.CALIB_HAND_EYE_TSAI,
    "PARK": cv2.CALIB_HAND_EYE_PARK,
    "HORAUD": cv2.CALIB_HAND_EYE_HORAUD,
    "ANDREFF": cv2.CALIB_HAND_EYE_ANDREFF,
    "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS,
}


def solve_hand_eye(T_base_gripper_list, T_cam_target_list, method="PARK"):
    """Calibracion mano-ojo (eye-in-hand): resuelve AX = XB y devuelve T_gripper_cam (4x4).

    Entradas: listas de matrices 4x4 con
        T_base_gripper  (cinematica del robot: base <- brida)
        T_cam_target    (pose del patron en la camara: camara <- patron)
    Ver cv::calibrateHandEye (OpenCV) y Tsai & Lenz (1989), Park & Martin (1994).
    """
    R_g2b = [T[:3, :3] for T in T_base_gripper_list]
    t_g2b = [T[:3, 3].reshape(3, 1) for T in T_base_gripper_list]
    R_t2c = [T[:3, :3] for T in T_cam_target_list]
    t_t2c = [T[:3, 3].reshape(3, 1) for T in T_cam_target_list]
    R, t = cv2.calibrateHandEye(R_g2b, t_g2b, R_t2c, t_t2c, method=HANDEYE_METHODS[method])
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, t.ravel()
    return T


def hand_eye_consistency(T_base_gripper_list, T_cam_target_list, T_gripper_cam):
    """Dispersion (m) de la posicion del patron en la base: debe ser casi constante si X es correcta."""
    pts = np.array([(Tb @ T_gripper_cam @ Tc)[:3, 3]
                    for Tb, Tc in zip(T_base_gripper_list, T_cam_target_list)])
    center = pts.mean(axis=0)
    err = np.linalg.norm(pts - center, axis=1)
    return center, float(err.mean()), float(err.max())
