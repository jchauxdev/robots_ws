"""Genera el mundo "laboratorio" a partir de una lista de rectangulos (unica fuente de verdad):
  * worlds/laboratorio.png + laboratorio.yaml : mapa de verdad terreno para el simulador de lidar 2D
  * worlds/laboratorio.sdf                    : el mismo mundo para Gazebo Fortress (con GPU)

Uso:  python3 -m mr_sim.world_gen <carpeta_salida>
Coordenadas en metros, marco "world" (origen en la esquina inferior izquierda del recinto).
"""
import sys
from pathlib import Path

import numpy as np

RES = 0.02          # m/pixel del mapa de verdad terreno
W, H = 12.0, 8.0    # recinto 12 m x 8 m
T = 0.10            # espesor de muros

# (nombre, xmin, ymin, xmax, ymax, altura)
OBSTACLES = [
    ('muro_sur', 0, 0, W, T, 1.0), ('muro_norte', 0, H - T, W, H, 1.0),
    ('muro_oeste', 0, 0, T, H, 1.0), ('muro_este', W - T, 0, W, H, 1.0),
    # tabique interior con puerta de 1.2 m (x = 6 m)
    ('tabique_a', 6.0, T, 6.1, 3.2, 1.0), ('tabique_b', 6.0, 4.4, 6.1, H - T, 1.0),
    # celda del cobot: mesa baja 1.2 x 0.7 x 0.30 m (bandeja del robot al alcance del brazo)
    ('mesa_cobot', 0.6, 6.4, 1.8, 7.1, 0.30),
    # estanterias y mesas
    ('estante_1', 2.8, 0.1, 4.6, 0.6, 1.8), ('mesa_central', 2.6, 3.4, 3.8, 4.4, 0.75),
    ('columna', 4.6, 6.0, 4.9, 6.3, 2.5),
    ('estante_2', 7.0, 7.3, 9.2, 7.9, 1.8), ('mesa_e', 9.6, 3.0, 10.6, 4.6, 0.75),
    ('caja_1', 8.0, 1.2, 8.5, 1.7, 0.5),
]

# Puntos de referencia en el marco world (x, y, yaw[rad]); la SLAM fija el marco map en la pose inicial.
LANDMARKS = {
    'inicio': (1.0, 1.0, 0.0),
    'cobot': (1.2, 6.15, -1.5708),      # de espaldas a la mesa: la bandeja trasera queda junto al cobot
    'estacion_A': (4.0, 2.0, 0.0),
    'estacion_B': (9.0, 6.0, 0.0),
    'estacion_C': (10.8, 1.0, -1.5708),
}


def occupancy():
    nx, ny = int(round(W / RES)), int(round(H / RES))
    img = np.full((ny, nx), 254, np.uint8)  # libre
    for _, x0, y0, x1, y1, _h in OBSTACLES:
        c0, c1 = int(round(x0 / RES)), int(round(x1 / RES))
        r0, r1 = int(round(y0 / RES)), int(round(y1 / RES))
        img[ny - r1:ny - r0, c0:c1] = 0          # filas invertidas: la imagen tiene y hacia abajo
    return img


def sdf():
    boxes = []
    for name, x0, y0, x1, y1, h in OBSTACLES:
        cx, cy, sx, sy = (x0 + x1) / 2, (y0 + y1) / 2, x1 - x0, y1 - y0
        boxes.append(f"""
    <model name="{name}"><static>true</static><pose>{cx:.3f} {cy:.3f} {h / 2:.3f} 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>{sx:.3f} {sy:.3f} {h:.3f}</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>{sx:.3f} {sy:.3f} {h:.3f}</size></box></geometry>
          <material><ambient>0.6 0.6 0.65 1</ambient><diffuse>0.7 0.7 0.75 1</diffuse></material></visual>
      </link></model>""")
    return f"""<?xml version="1.0"?>
<!-- Generado por mr_sim/world_gen.py: mismo recinto que worlds/laboratorio.png -->
<sdf version="1.8">
  <world name="laboratorio">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size><real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>
    <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"/>
    <plugin filename="ignition-gazebo-sensors-system" name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <light type="directional" name="sol"><cast_shadows>true</cast_shadows><pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse><direction>-0.5 0.1 -0.9</direction></light>
    <model name="piso"><static>true</static>
      <link name="link">
        <collision name="c"><geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry></collision>
        <visual name="v"><geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry>
          <material><ambient>0.8 0.8 0.8 1</ambient><diffuse>0.8 0.8 0.8 1</diffuse></material></visual>
      </link></model>{''.join(boxes)}
  </world>
</sdf>
"""


def main():
    out = Path(sys.argv[1] if len(sys.argv) > 1 else 'worlds')
    out.mkdir(parents=True, exist_ok=True)
    from PIL import Image
    Image.fromarray(occupancy()).save(out / 'laboratorio.png')
    (out / 'laboratorio.yaml').write_text(
        f"image: laboratorio.png\nresolution: {RES}\norigin: [0.0, 0.0, 0.0]\nnegate: 0\n"
        "occupied_thresh: 0.65\nfree_thresh: 0.25\nmode: trinary\n")
    (out / 'laboratorio.sdf').write_text(sdf())
    import yaml
    (out / 'landmarks.yaml').write_text(yaml.safe_dump(
        {k: {'x': v[0], 'y': v[1], 'yaw': v[2]} for k, v in LANDMARKS.items()}, sort_keys=False))
    print('Mundo generado en', out)


if __name__ == '__main__':
    main()
