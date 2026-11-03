"""Practica 5.3 - Entrenamiento de un DETECTOR propio (Ultralytics YOLO, PyTorch) y exportacion.

Dataset en formato YOLO (ver frutas.yaml): images/{train,val}, labels/{train,val} con
una linea por objeto: <clase> <xc> <yc> <w> <h> (normalizados 0-1). Etiquetado con CVAT,
Label Studio o Roboflow.

    python3 dl_03_yolo_entrenar.py --datos frutas.yaml --modelo yolo11n.pt --epocas 100
    python3 dl_03_yolo_entrenar.py --exportar runs/detect/frutas/weights/best.pt   # en la Jetson
"""
import argparse

from ultralytics import YOLO


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--datos", default="frutas.yaml")
    ap.add_argument("--modelo", default="yolo11n.pt", help="pesos preentrenados (transfer learning)")
    ap.add_argument("--epocas", type=int, default=100)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--lote", type=int, default=16)
    ap.add_argument("--dispositivo", default="", help="'' auto | cpu | 0")
    ap.add_argument("--nombre", default="frutas")
    ap.add_argument("--exportar", default="", help="ruta a best.pt para exportar a TensorRT")
    a = ap.parse_args()

    if a.exportar:
        m = YOLO(a.exportar)
        ruta = m.export(format="engine", half=True, imgsz=a.imgsz)  # TensorRT FP16 (en la Jetson)
        print("Motor TensorRT:", ruta)
        return

    m = YOLO(a.modelo)
    m.train(data=a.datos, epochs=a.epocas, imgsz=a.imgsz, batch=a.lote, device=a.dispositivo or None,
            name=a.nombre, patience=30, seed=42, deterministic=True, plots=True)
    met = m.val(data=a.datos, imgsz=a.imgsz, split="val")
    print(f"mAP50 = {met.box.map50:.3f}   mAP50-95 = {met.box.map:.3f}")
    for i, n in m.names.items():
        try:
            print(f"  {n:12s} AP50-95 = {met.box.maps[i]:.3f}")
        except Exception:
            pass
    m.export(format="onnx", imgsz=a.imgsz)


if __name__ == "__main__":
    main()
