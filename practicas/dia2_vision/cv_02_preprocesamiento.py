"""Practica 4.2 - Preprocesamiento: espacios de color, filtrado, ecualizacion (CLAHE) y bordes.

    python3 cv_02_preprocesamiento.py --fuente 0
Muestra un mosaico: original | gris | Gauss | mediana | CLAHE | Canny.
"""
import cv2
import numpy as np

from comun import abrir_fuente, argumentos


def mosaico(imgs, ancho=320):
    out = []
    for titulo, im in imgs:
        if im.ndim == 2:
            im = cv2.cvtColor(im, cv2.COLOR_GRAY2BGR)
        im = cv2.resize(im, (ancho, int(im.shape[0] * ancho / im.shape[1])))
        cv2.putText(im, titulo, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
        out.append(im)
    filas = [np.hstack(out[i:i + 3]) for i in range(0, len(out), 3)]
    return np.vstack(filas)


def procesar(frame, t1=60, t2=150):
    gris = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gauss = cv2.GaussianBlur(gris, (5, 5), 1.2)
    mediana = cv2.medianBlur(gris, 5)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(gris)
    bordes = cv2.Canny(gauss, t1, t2)
    return [("original", frame), ("gris", gris), ("Gauss 5x5", gauss),
            ("mediana 5", mediana), ("CLAHE", clahe), (f"Canny {t1}/{t2}", bordes)]


def main():
    a = argumentos(__doc__).parse_args()
    leer, liberar = abrir_fuente(a.fuente, a.ancho, a.alto)
    cv2.namedWindow("preproc")
    cv2.createTrackbar("Canny t1", "preproc", 60, 255, lambda v: None)
    cv2.createTrackbar("Canny t2", "preproc", 150, 255, lambda v: None)
    while True:
        ok, frame = leer()
        if not ok:
            break
        t1, t2 = cv2.getTrackbarPos("Canny t1", "preproc"), cv2.getTrackbarPos("Canny t2", "preproc")
        cv2.imshow("preproc", mosaico(procesar(frame, t1, t2)))
        if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
            break
    liberar()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
