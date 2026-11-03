"""Detector por COLOR (HSV) y FORMA (aproximacion poligonal) - Día 3 (simulación, DA-10) y Día 4 (robot real, DA-16).

Perfiles en YAML (param profiles_file), p. ej. config/hsv_profiles.yaml:
  rojo:  {ranges: [[[0,120,70],[10,255,255]], [[170,120,70],[179,255,255]]], min_area: 300}
"""
import cv2
import yaml

from jc_vision.detector_base import DetectorBase, spin
from jc_vision.vision_algorithms import find_blobs, hsv_mask


class ColorShapeDetector(DetectorBase):
    def __init__(self):
        super().__init__("color_shape_detector")
        path = self.declare_parameter("profiles_file", "").value
        self.max_area = float(self.declare_parameter("max_area", 40000.0).value)
        with open(path, encoding="utf-8") as f:
            self.profiles = yaml.safe_load(f)
        self.get_logger().info(f"Perfiles HSV: {list(self.profiles)}")

    def detect(self, bgr):
        dets = []
        for label, prof in self.profiles.items():
            mask = hsv_mask(bgr, prof["ranges"])
            for b in find_blobs(mask, prof.get("min_area", 300), self.max_area):
                b["label"] = label
                dets.append(b)
        return dets

    def draw(self, img, dets):
        for d in dets:
            cv2.drawContours(img, [d["contour"]], -1, (0, 255, 255), 2)
            cv2.circle(img, (int(d["u"]), int(d["v"])), 4, (255, 255, 255), -1)
            cv2.putText(img, f'{d["label"]}/{d["shape"]}', (int(d["u"]) + 6, int(d["v"]) - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        return img


def main():
    spin(ColorShapeDetector)
