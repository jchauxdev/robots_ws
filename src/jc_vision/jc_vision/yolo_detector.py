"""Detector con red neuronal (Ultralytics YOLO sobre PyTorch / TensorRT) - Dias 4-5.

Parametros: model (yolo11n.pt | best.pt | best.engine), conf, classes (lista de nombres
a conservar; vacia = todas), device ('' auto, 'cpu', 0), imgsz.
La posicion 3D se estima intersectando el centro de la caja con el plano z = plane_z.
"""
import cv2

from jc_vision.detector_base import DetectorBase, spin


class YoloDetector(DetectorBase):
    def __init__(self):
        super().__init__("yolo_detector")
        p = self.declare_parameter
        self.model_path = p("model", "yolo11n.pt").value
        self.conf = float(p("conf", 0.5).value)
        self.keep = [c for c in p("classes", [""]).value if c]
        self.device = str(p("device", "").value)
        self.imgsz = int(p("imgsz", 640).value)
        try:
            from ultralytics import YOLO
        except ImportError as e:
            raise SystemExit("Instale ultralytics: pip install ultralytics (ver DA-11)") from e
        self.model = YOLO(self.model_path)
        self.get_logger().info(f"Modelo {self.model_path}; clases: {self.keep or 'todas'}")

    def detect(self, bgr):
        kw = {"conf": self.conf, "imgsz": self.imgsz, "verbose": False}
        if self.device:
            kw["device"] = self.device
        r = self.model.predict(bgr, **kw)[0]
        dets = []
        for box, cls, sc in zip(r.boxes.xywh.tolist(), r.boxes.cls.tolist(), r.boxes.conf.tolist()):
            name = r.names[int(cls)]
            if self.keep and name not in self.keep:
                continue
            x, y, w, h = box
            dets.append({"label": name, "shape": "", "score": sc, "u": x, "v": y, "w": w, "h": h,
                         "angle_rad": 0.0})
        return dets

    def draw(self, img, dets):
        for d in dets:
            x0, y0 = int(d["u"] - d["w"] / 2), int(d["v"] - d["h"] / 2)
            x1, y1 = int(d["u"] + d["w"] / 2), int(d["v"] + d["h"] / 2)
            cv2.rectangle(img, (x0, y0), (x1, y1), (0, 200, 0), 2)
            cv2.putText(img, f'{d["label"]} {d["score"]:.2f}', (x0, max(12, y0 - 4)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 0), 1, cv2.LINE_AA)
        return img


def main():
    spin(YoloDetector)
