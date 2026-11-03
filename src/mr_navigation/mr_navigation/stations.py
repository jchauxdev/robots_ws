"""Lectura/escritura del archivo de estaciones (puntos con nombre) sin dependencias de ROS.

Formato YAML (marco 'map', metros y grados para que sea facil de editar a mano):
  frame_id: map
  estaciones:
    cobot:      {x: 0.20, y: 5.15, yaw_deg: -90.0, descripcion: "Atraque frente al JetCobot"}
    estacion_A: {x: 3.00, y: 1.00, yaw_deg: 0.0,   descripcion: "Entrega A"}
"""
import math
import re
from pathlib import Path
from typing import Dict, Tuple, Union

import yaml

NAME_RE = re.compile(r'^[A-Za-z0-9_\-]{1,40}$')


class Station:
    """Punto con nombre (x, y en m; yaw en rad)."""

    def __init__(self, name: str, x: float, y: float, yaw: float, description: str = ''):
        self.name, self.x, self.y, self.yaw, self.description = name, x, y, yaw, description

    def __eq__(self, other):
        return isinstance(other, Station) and vars(self) == vars(other)

    def __repr__(self):
        return 'Station({!r}, x={:.3f}, y={:.3f}, yaw={:.3f})'.format(self.name, self.x, self.y, self.yaw)

    def to_dict(self):
        return {'x': round(self.x, 3), 'y': round(self.y, 3), 'yaw_deg': round(math.degrees(self.yaw), 1),
                'descripcion': self.description}


def valid_name(name: str) -> bool:
    return bool(NAME_RE.match(name or ''))


def load(path: Union[str, Path]) -> Tuple[str, Dict[str, Station]]:
    p = Path(path)
    if not p.exists():
        return 'map', {}
    data = yaml.safe_load(p.read_text()) or {}
    frame = data.get('frame_id', 'map')
    out = {}
    for name, d in (data.get('estaciones') or {}).items():
        yaw = math.radians(float(d.get('yaw_deg', 0.0))) if 'yaw_deg' in d else float(d.get('yaw', 0.0))
        out[str(name)] = Station(str(name), float(d['x']), float(d['y']), yaw, str(d.get('descripcion', '')))
    return frame, out


def save(path: Union[str, Path], frame: str, stations: Dict[str, Station]):
    data = {'frame_id': frame, 'estaciones': {n: s.to_dict() for n, s in stations.items()}}
    Path(path).write_text('# Estaciones del robot movil (generado/actualizado por station_manager)\n' +
                          yaml.safe_dump(data, sort_keys=False, allow_unicode=True))
