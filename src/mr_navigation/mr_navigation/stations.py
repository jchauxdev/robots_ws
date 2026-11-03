"""Lectura/escritura del archivo de estaciones (puntos con nombre) sin dependencias de ROS.

Formato YAML (marco 'map', metros y grados para que sea facil de editar a mano):
  frame_id: map
  estaciones:
    cobot:      {x: 0.20, y: 5.15, yaw_deg: -90.0, descripcion: "Atraque frente al JetCobot"}
    estacion_A: {x: 3.00, y: 1.00, yaw_deg: 0.0,   descripcion: "Entrega A"}
"""
from __future__ import annotations

import math
import re
from dataclasses import dataclass
from pathlib import Path

import yaml

NAME_RE = re.compile(r'^[A-Za-z0-9_\-]{1,40}$')


@dataclass
class Station:
    name: str
    x: float
    y: float
    yaw: float            # rad
    description: str = ''

    def to_dict(self):
        return {'x': round(self.x, 3), 'y': round(self.y, 3), 'yaw_deg': round(math.degrees(self.yaw), 1),
                'descripcion': self.description}


def valid_name(name: str) -> bool:
    return bool(NAME_RE.match(name or ''))


def load(path: str | Path) -> tuple[str, dict[str, Station]]:
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


def save(path: str | Path, frame: str, stations: dict[str, Station]):
    data = {'frame_id': frame, 'estaciones': {n: s.to_dict() for n, s in stations.items()}}
    Path(path).write_text('# Estaciones del robot movil (generado/actualizado por station_manager)\n' +
                          yaml.safe_dump(data, sort_keys=False, allow_unicode=True))
