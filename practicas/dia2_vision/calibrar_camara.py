"""Practica 4.5 - Calibracion intrinseca con tablero de ajedrez (metodo de Zhang, 2000).

1) Captura:   python3 calibrar_camara.py capturar --fuente /dev/video0 --carpeta calib
              (barra espaciadora = guardar cuando el tablero se detecta; 15-25 vistas variadas)
2) Calibrar:  python3 calibrar_camara.py calibrar --carpeta calib --esquinas 8x6 --cuadro 0.025
Salida: camera_info.yaml (formato ROS), error RMS de reproyeccion y error por imagen.
"""
import argparse
import glob
import os

import cv2
import numpy as np
import yaml

CRIT = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)


def capturar(a):
    from comun import abrir_fuente
    os.makedirs(a.carpeta, exist_ok=True)
    cols, filas = map(int, a.esquinas.split("x"))
    leer, liberar = abrir_fuente(a.fuente)
    n = len(glob.glob(os.path.join(a.carpeta, "*.png")))
    while True:
        ok, frame = leer()
        if not ok:
            break
        gris = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        enc, esq = cv2.findChessboardCorners(gris, (cols, filas), cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK)
        vis = frame.copy()
        if enc:
            cv2.drawChessboardCorners(vis, (cols, filas), esq, enc)
        cv2.putText(vis, f"vistas: {n}  (espacio=guardar, q=salir)", (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.imshow("captura", vis)
        k = cv2.waitKey(1) & 0xFF
        if k == ord(" ") and enc:
            n += 1
            cv2.imwrite(os.path.join(a.carpeta, f"vista_{n:02d}.png"), frame)
        if k in (ord("q"), 27):
            break
    liberar()
    cv2.destroyAllWindows()


def calibrar(a):
    cols, filas = map(int, a.esquinas.split("x"))
    objp = np.zeros((cols * filas, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:filas].T.reshape(-1, 2) * a.cuadro
    obj, img, nombres, tam = [], [], [], None
    for f in sorted(glob.glob(os.path.join(a.carpeta, "*.png")) + sorted(glob.glob(os.path.join(a.carpeta, "*.jpg")))):
        gris = cv2.cvtColor(cv2.imread(f), cv2.COLOR_BGR2GRAY)
        tam = gris.shape[::-1]
        enc, esq = cv2.findChessboardCorners(gris, (cols, filas), None)
        if not enc:
            print(f"  sin tablero: {os.path.basename(f)}")
            continue
        esq = cv2.cornerSubPix(gris, esq, (11, 11), (-1, -1), CRIT)
        obj.append(objp)
        img.append(esq)
        nombres.append(os.path.basename(f))
    if len(obj) < 8:
        raise SystemExit(f"Solo {len(obj)} vistas validas; se recomiendan >= 15")
    rms, K, D, rvecs, tvecs = cv2.calibrateCamera(obj, img, tam, None, None)
    print(f"Vistas usadas: {len(obj)}   RMS reproyeccion = {rms:.3f} px")
    for n, o, i, r, t in zip(nombres, obj, img, rvecs, tvecs):
        p, _ = cv2.projectPoints(o, r, t, K, D)
        e = np.sqrt(np.mean(np.sum((p.reshape(-1, 2) - i.reshape(-1, 2)) ** 2, axis=1)))
        print(f"  {n:20s} error = {e:.3f} px")
    fov_h = 2 * np.degrees(np.arctan(tam[0] / (2 * K[0, 0])))
    print("K =\n", np.round(K, 3), "\nD =", np.round(D.ravel(), 5), f"\nFOV horizontal = {fov_h:.1f} grados")
    P = np.hstack([K, np.zeros((3, 1))])
    data = {"image_width": tam[0], "image_height": tam[1], "camera_name": "wrist_camera",
            "camera_matrix": {"rows": 3, "cols": 3, "data": K.ravel().tolist()},
            "distortion_model": "plumb_bob",
            "distortion_coefficients": {"rows": 1, "cols": 5, "data": D.ravel()[:5].tolist()},
            "rectification_matrix": {"rows": 3, "cols": 3, "data": np.eye(3).ravel().tolist()},
            "projection_matrix": {"rows": 3, "cols": 4, "data": P.ravel().tolist()},
            "rms_reprojection_error_px": float(rms)}
    with open(a.salida, "w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, sort_keys=False)
    print(f"Guardado {a.salida}")
    return rms, K, D


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("modo", choices=["capturar", "calibrar"])
    ap.add_argument("--fuente", default="0")
    ap.add_argument("--carpeta", default="calib")
    ap.add_argument("--esquinas", default="8x6", help="esquinas INTERNAS columnas x filas")
    ap.add_argument("--cuadro", type=float, default=0.025, help="lado del cuadro en metros")
    ap.add_argument("--salida", default="camera_info.yaml")
    a = ap.parse_args()
    capturar(a) if a.modo == "capturar" else calibrar(a)


if __name__ == "__main__":
    main()
