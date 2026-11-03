"""Practica 4.4 - Extraccion de caracteristicas: contornos, momentos (centroide, Hu),
area, perimetro, circularidad, rectangulo de area minima y clasificacion de forma.
Tambien compara detectores de puntos clave ORB y SIFT (OpenCV >= 4.4).

    python3 cv_04_caracteristicas.py --fuente 0          # en vivo
    python3 cv_04_caracteristicas.py --fuente piezas.png --tabla   # imprime tabla de rasgos
"""
import math

import cv2
import numpy as np

from comun import abrir_fuente, argumentos


def clasificar(c):
    peri = cv2.arcLength(c, True)
    area = cv2.contourArea(c)
    aprox = cv2.approxPolyDP(c, 0.035 * peri, True)
    circ = 4 * math.pi * area / (peri * peri) if peri else 0
    if len(aprox) == 3:
        return "triangulo", circ
    if len(aprox) == 4:
        (_, _), (w, h), _ = cv2.minAreaRect(c)
        return ("cuadrado" if max(w, h) / max(1e-6, min(w, h)) < 1.2 else "rectangulo"), circ
    return ("circulo" if circ > 0.8 else f"poligono_{len(aprox)}"), circ


def rasgos(frame, area_min=300):
    gris = cv2.GaussianBlur(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), (5, 5), 0)
    _, mask = cv2.threshold(gris, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    cs, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    filas = []
    for c in cs:
        area = cv2.contourArea(c)
        if area < area_min:
            continue
        m = cv2.moments(c)
        cx, cy = m["m10"] / m["m00"], m["m01"] / m["m00"]
        hu = cv2.HuMoments(m).ravel()
        hu_log = -np.sign(hu) * np.log10(np.abs(hu) + 1e-30)
        forma, circ = clasificar(c)
        (rx, ry), (w, h), ang = cv2.minAreaRect(c)
        filas.append({"forma": forma, "cx": cx, "cy": cy, "area": area, "perimetro": cv2.arcLength(c, True),
                      "circularidad": circ, "angulo": ang, "hu1": hu_log[0], "hu2": hu_log[1], "contorno": c,
                      "caja": cv2.boxPoints(((rx, ry), (w, h), ang))})
    return filas


def main():
    ap = argumentos(__doc__)
    ap.add_argument("--tabla", action="store_true")
    a = ap.parse_args()
    leer, liberar = abrir_fuente(a.fuente, a.ancho, a.alto)
    orb = cv2.ORB_create(500)
    while True:
        ok, frame = leer()
        if not ok:
            break
        filas = rasgos(frame)
        if a.tabla:
            print(f"{'forma':12s} {'cx':>7s} {'cy':>7s} {'area':>8s} {'perim':>7s} {'circ':>5s} {'ang':>6s} {'Hu1':>6s} {'Hu2':>6s}")
            for f in filas:
                print(f"{f['forma']:12s} {f['cx']:7.1f} {f['cy']:7.1f} {f['area']:8.0f} {f['perimetro']:7.1f} "
                      f"{f['circularidad']:5.2f} {f['angulo']:6.1f} {f['hu1']:6.2f} {f['hu2']:6.2f}")
            break
        vis = frame.copy()
        for f in filas:
            cv2.drawContours(vis, [f["contorno"]], -1, (0, 255, 0), 2)
            cv2.drawContours(vis, [np.int32(f["caja"])], -1, (255, 0, 0), 1)
            cv2.circle(vis, (int(f["cx"]), int(f["cy"])), 4, (0, 0, 255), -1)
            cv2.putText(vis, f"{f['forma']} c={f['circularidad']:.2f}", (int(f["cx"]) + 6, int(f["cy"])),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        kp = orb.detect(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), None)
        vis = cv2.drawKeypoints(vis, kp, None, color=(0, 200, 255))
        cv2.imshow("caracteristicas (contornos + ORB)", vis)
        if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
            break
    liberar()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
