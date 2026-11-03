"""Practica 5.0 - Modelo PREENTRENADO (COCO) para detectar frutas sin entrenar.

El conjunto COCO (Lin et al., 2014) incluye 80 clases, entre ellas: banana, apple, orange,
broccoli, carrot. Este script filtra esas clases.
    python3 dl_00_yolo_preentrenado.py --fuente 0 --modelo yolo11n.pt
    python3 dl_00_yolo_preentrenado.py --fuente foto.jpg --guardar salida.jpg
"""
import argparse
import time

import cv2
from ultralytics import YOLO

FRUTAS = {"banana", "apple", "orange", "broccoli", "carrot"}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fuente", default="0")
    ap.add_argument("--modelo", default="yolo11n.pt")
    ap.add_argument("--conf", type=float, default=0.4)
    ap.add_argument("--todas", action="store_true", help="no filtrar clases")
    ap.add_argument("--guardar", default="")
    a = ap.parse_args()
    modelo = YOLO(a.modelo)  # descarga automatica de pesos la primera vez
    ids = None if a.todas else [k for k, v in modelo.names.items() if v in FRUTAS]
    print("Clases filtradas:", "todas" if ids is None else {i: modelo.names[i] for i in ids})

    if a.fuente.lower().endswith((".jpg", ".jpeg", ".png")):
        r = modelo.predict(a.fuente, conf=a.conf, classes=ids, verbose=False)[0]
        for b in r.boxes:
            print(f"{r.names[int(b.cls)]:10s} conf={float(b.conf):.2f} caja={[round(x) for x in b.xyxy[0].tolist()]}")
        if a.guardar:
            cv2.imwrite(a.guardar, r.plot())
        return

    cap = cv2.VideoCapture(int(a.fuente) if a.fuente.isdigit() else a.fuente)
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        t = time.perf_counter()
        r = modelo.predict(frame, conf=a.conf, classes=ids, verbose=False)[0]
        dt = (time.perf_counter() - t) * 1000
        vis = r.plot()
        cv2.putText(vis, f"{dt:.1f} ms/imagen", (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.imshow("YOLO preentrenado (COCO)", vis)
        if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
            break
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
