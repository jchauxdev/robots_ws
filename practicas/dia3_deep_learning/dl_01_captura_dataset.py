"""Practica 5.1 - Captura de imagenes para un dataset propio (misma camara del robot).

    python3 dl_01_captura_dataset.py --fuente /dev/video0 --clase manzana --salida dataset_raw
Teclas: espacio = guardar, a = modo rafaga (1 imagen cada 0.5 s), q = salir.
Buenas practicas: variar posicion, orientacion, iluminacion y fondo; incluir oclusiones
parciales y varias piezas por imagen; >= 150 instancias por clase como punto de partida.
"""
import argparse
import os
import time

import cv2


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fuente", default="0")
    ap.add_argument("--clase", required=True)
    ap.add_argument("--salida", default="dataset_raw")
    a = ap.parse_args()
    carpeta = os.path.join(a.salida, a.clase)
    os.makedirs(carpeta, exist_ok=True)
    cap = cv2.VideoCapture(int(a.fuente) if a.fuente.isdigit() else a.fuente)
    n, rafaga, t_last = len(os.listdir(carpeta)), False, 0.0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        vis = frame.copy()
        cv2.putText(vis, f"{a.clase}: {n} imagenes {'[RAFAGA]' if rafaga else ''}", (10, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow("captura dataset", vis)
        k = cv2.waitKey(1) & 0xFF
        guardar = k == ord(" ") or (rafaga and time.time() - t_last > 0.5)
        if guardar:
            n += 1
            t_last = time.time()
            cv2.imwrite(os.path.join(carpeta, f"{a.clase}_{int(time.time()*1000)}.jpg"), frame)
        if k == ord("a"):
            rafaga = not rafaga
        if k in (ord("q"), 27):
            break
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
