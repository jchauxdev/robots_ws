# robots_ws – espacio de trabajo ROS 2 Humble (SENA 2026)

Paquetes ROS 2 de las dos transferencias de conocimiento del SENA (Julián René Cháux Cedeño):

- **Robot móvil** (`mr_*`): RoboClaw, modelo, lanzadores, cámara, navegación, logística y enlace con el cobot.
- **Celda del cobot JetCobot** (`jc_*`): interfaces, modelo, MoveIt 2, movimiento, visión, driver y aplicaciones.

Los documentos de apoyo (DA) están en el repositorio del curso: <https://github.com/jchauxdev/JetCobot>.

## Cómo se usa: un paquete a la vez

Este repositorio **es** el espacio de trabajo `~/robots_ws`. Se clona una vez vacío y cada documento de apoyo, en su
**Paso 0**, descarga solo los paquetes que va a usar. Para recibir las correcciones del instructor basta con `git pull`.

### 1. Clonar el espacio de trabajo vacío (una vez por equipo)

**Terminal – PC y computadores del robot con Ubuntu 22.04 (Orin Nano, AGX Orin, Raspberry Pi)**

```bash
sudo apt install -y git
git clone --filter=blob:none --sparse https://github.com/jchauxdev/robots_ws.git ~/robots_ws
cd ~/robots_ws && colcon build
```

La Jetson Nano (Ubuntu 18.04, git 2.17) usa otro método; está explicado en su DA-11.2.

### 2. Agregar paquetes (Paso 0 de cada DA)

**Terminal – el equipo que indique el DA**

```bash
cd ~/robots_ws
git sparse-checkout add src/mr_base src/mr_sim
colcon build --symlink-install --packages-select mr_base mr_sim
source install/setup.bash
```

### 3. Actualizar con los cambios del instructor

**Terminal – cada equipo (PC y robot)**

```bash
cd ~/robots_ws
git pull
colcon build --symlink-install
```

Si `git pull` avisa que sus cambios locales se sobrescribirían, guárdelos con `git stash`, ejecute `git pull` y
recupérelos con `git stash pop`.

## Contenido

| Carpeta | Contenido | Se agrega en |
|---|---|---|
| `src/mr_base`, `src/mr_sim` | Driver del RoboClaw y robot virtual | DA-9.1 (PC) y DA-11.x (robot) |
| `src/mr_description`, `src/mr_bringup` | Modelo URDF y lanzadores | DA-12.1 |
| `src/mr_perception` | Cámara USB con OpenCV | DA-15.1 |
| `src/mr_interfaces`, `src/mr_navigation` | Mensajes propios y puntos con nombre | DA-18.1 |
| `src/mr_logistics` | Gestor de pedidos y App web | DA-19.1 |
| `src/mr_cobot_bridge` | Enlace robot móvil – cobot | DA-21.1 |
| `docker/` | Contenedor ROS 2 de la Jetson Nano | DA-11.2 |
| `systemd/` | Arranque automático de los drivers | DA-13.3 |
| `src/jc_interfaces`, `src/jc_apps`, `src/jc_description` | Interfaces, aplicaciones y modelo del cobot | DA-03 |
| `src/jc_moveit_config`, `src/jc_motion`, `src/jc_bringup` | MoveIt 2, movimiento y lanzadores | DA-05 |
| `tools/` | Herramientas de simulación y calibración | DA-06 |
| `practicas/dia2_vision` | Prácticas de OpenCV | DA-08 |
| `src/jc_vision` | Visión en ROS 2 | DA-09 |
| `practicas/dia3_deep_learning` | Prácticas de PyTorch / YOLO | DA-11 |
| `src/jc_driver` | Driver del JetCobot real | DA-14 |

Licencia Apache-2.0.
