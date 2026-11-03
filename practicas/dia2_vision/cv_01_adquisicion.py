"""Practica 4.1 - Adquisicion: abrir la camara, medir FPS real, consultar propiedades V4L2.

    python3 cv_01_adquisicion.py --fuente /dev/video0
Teclas: s = guardar imagen, q/ESC = salir.
"""
import time

import cv2

from comun import abrir_fuente, argumentos


def main():
    a = argumentos(__doc__).parse_args()
    leer, liberar = abrir_fuente(a.fuente, a.ancho, a.alto)
    n, t0, fps, k = 0, time.time(), 0.0, 0
    while True:
        ok, frame = leer()
        if not ok:
            print("Fallo de captura")
            break
        n += 1
        if time.time() - t0 >= 1.0:
            fps, n, t0 = n / (time.time() - t0), 0, time.time()
        h, w = frame.shape[:2]
        cv2.putText(frame, f"{w}x{h}  {fps:.1f} FPS  dtype={frame.dtype}", (10, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.imshow("adquisicion", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("s"):
            k += 1
            cv2.imwrite(f"captura_{k:03d}.png", frame)
            print(f"guardada captura_{k:03d}.png")
        if key in (ord("q"), 27):
            break
    liberar()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
