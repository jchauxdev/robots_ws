"""Utilidades comunes de las practicas de vision (fuente de imagenes)."""
import argparse

import cv2


def argumentos(desc):
    ap = argparse.ArgumentParser(description=desc)
    ap.add_argument("--fuente", default="0",
                    help="indice de camara (0), dispositivo (/dev/video0), video o imagen")
    ap.add_argument("--ancho", type=int, default=640)
    ap.add_argument("--alto", type=int, default=480)
    return ap


def abrir_fuente(fuente, ancho=640, alto=480):
    """Devuelve una funcion leer() -> (ok, frame) para camara, video o imagen fija."""
    if fuente.lower().endswith((".png", ".jpg", ".jpeg", ".bmp")):
        img = cv2.imread(fuente)
        if img is None:
            raise FileNotFoundError(fuente)
        return lambda: (True, img.copy()), lambda: None
    src = int(fuente) if fuente.isdigit() else fuente
    cap = cv2.VideoCapture(src, cv2.CAP_V4L2) if str(src).startswith("/dev/") else cv2.VideoCapture(src)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, ancho)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, alto)
    if not cap.isOpened():
        raise RuntimeError(f"No se pudo abrir {fuente}")
    return cap.read, cap.release
