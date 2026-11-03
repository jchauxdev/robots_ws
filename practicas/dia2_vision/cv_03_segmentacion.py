"""Practica 4.3 - Segmentacion: umbral HSV con barras, Otsu y morfologia; exporta el perfil a YAML.

    python3 cv_03_segmentacion.py --fuente 0 --nombre rojo
Ajuste las barras hasta aislar la pieza; tecla 'g' guarda el rango en perfil_<nombre>.yaml
(formato de jc_vision/config/hsv_profiles.yaml). Teclas: o = alternar Otsu, q = salir.
"""
import cv2
import numpy as np
import yaml

from comun import abrir_fuente, argumentos


def main():
    ap = argumentos(__doc__)
    ap.add_argument("--nombre", default="pieza")
    a = ap.parse_args()
    leer, liberar = abrir_fuente(a.fuente, a.ancho, a.alto)
    w = "segmentacion"
    cv2.namedWindow(w)
    for n, v, m in [("H min", 0, 179), ("H max", 10, 179), ("S min", 120, 255), ("S max", 255, 255),
                    ("V min", 70, 255), ("V max", 255, 255), ("apertura", 1, 5), ("cierre", 2, 5)]:
        cv2.createTrackbar(n, w, v, m, lambda x: None)
    otsu = False
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    while True:
        ok, frame = leer()
        if not ok:
            break
        g = {n: cv2.getTrackbarPos(n, w) for n in ["H min", "H max", "S min", "S max", "V min", "V max", "apertura", "cierre"]}
        if otsu:
            gris = cv2.GaussianBlur(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), (5, 5), 0)
            th, mask = cv2.threshold(gris, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
            titulo = f"Otsu (umbral={th:.0f})"
        else:
            hsv = cv2.cvtColor(cv2.GaussianBlur(frame, (5, 5), 0), cv2.COLOR_BGR2HSV)
            lo = np.array([g["H min"], g["S min"], g["V min"]])
            hi = np.array([g["H max"], g["S max"], g["V max"]])
            mask = cv2.inRange(hsv, lo, hi)
            titulo = f"HSV {lo.tolist()}-{hi.tolist()}"
        if g["apertura"]:
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k, iterations=g["apertura"])
        if g["cierre"]:
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=g["cierre"])
        res = cv2.bitwise_and(frame, frame, mask=mask)
        cv2.putText(res, titulo, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
        cv2.imshow(w, np.hstack([frame, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR), res]))
        key = cv2.waitKey(1) & 0xFF
        if key == ord("o"):
            otsu = not otsu
        if key == ord("g") and not otsu:
            perfil = {a.nombre: {"ranges": [[lo.tolist(), hi.tolist()]], "min_area": 250}}
            with open(f"perfil_{a.nombre}.yaml", "w", encoding="utf-8") as f:
                yaml.safe_dump(perfil, f, default_flow_style=None)
            print(f"Guardado perfil_{a.nombre}.yaml:", perfil)
        if key in (ord("q"), 27):
            break
    liberar()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
