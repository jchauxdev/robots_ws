"""Genera patrones imprimibles: tablero de ajedrez (calibracion) y marcadores ArUco.

    python3 generar_patrones.py --tablero 9x7 --cuadro_mm 25 --aruco 0-9 --lado_mm 30
Imprimir al 100 % (sin 'ajustar a pagina') y VERIFICAR con calibrador el tamano real.
"""
import argparse

import cv2
import numpy as np

DPI = 300
MM = DPI / 25.4  # pixeles por mm


def tablero(cols, filas, cuadro_mm):
    s = int(round(cuadro_mm * MM))
    img = np.full(((filas + 2) * s, (cols + 2) * s), 255, np.uint8)
    for r in range(filas):
        for c in range(cols):
            if (r + c) % 2 == 0:
                img[(r + 1) * s:(r + 2) * s, (c + 1) * s:(c + 2) * s] = 0
    return img


def aruco(ids, lado_mm, dic="DICT_4X4_50"):
    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dic))
    s = int(round(lado_mm * MM))
    m = int(round(10 * MM))
    hojas = []
    for i in ids:
        mk = cv2.aruco.generateImageMarker(d, i, s) if hasattr(cv2.aruco, "generateImageMarker") \
            else cv2.aruco.drawMarker(d, i, s)
        mk = cv2.copyMakeBorder(mk, m, m + 60, m, m, cv2.BORDER_CONSTANT, value=255)
        cv2.putText(mk, f"{dic} id={i}  {lado_mm} mm", (m, mk.shape[0] - 25), cv2.FONT_HERSHEY_SIMPLEX, 1.2, 0, 2)
        hojas.append(mk)
    return hojas


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tablero", default="9x7", help="cuadros columnas x filas (esquinas internas = -1)")
    ap.add_argument("--cuadro_mm", type=float, default=25.0)
    ap.add_argument("--aruco", default="0-9")
    ap.add_argument("--lado_mm", type=float, default=30.0)
    a = ap.parse_args()
    c, f = map(int, a.tablero.lower().split("x"))
    cv2.imwrite(f"tablero_{c}x{f}_{a.cuadro_mm:g}mm_300dpi.png", tablero(c, f, a.cuadro_mm))
    print(f"Tablero: {c}x{f} cuadros -> esquinas internas {c-1}x{f-1}")
    lo, hi = map(int, a.aruco.split("-"))
    for i, h in zip(range(lo, hi + 1), aruco(range(lo, hi + 1), a.lado_mm)):
        cv2.imwrite(f"aruco_4x4_id{i}_{a.lado_mm:g}mm_300dpi.png", h)
    print(f"Marcadores {lo}..{hi} generados")


if __name__ == "__main__":
    main()
