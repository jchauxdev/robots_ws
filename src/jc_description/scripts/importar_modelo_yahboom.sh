#!/usr/bin/env bash
# Instala las mallas STL del JetCobot de Yahboom en jc_description/meshes/yahboom/.
# Son archivos de Yahboom (paquete jetcobot_description de la imagen de fabrica del robot); Yahboom no
# les declaro licencia, por eso NO se publican en el repositorio robots_ws: cada equipo los copia aqui.
#
# Uso:
#   importar_modelo_yahboom.sh <carpeta jetcobot_description>
#       carpeta del paquete de Yahboom: en la imagen de fabrica del JetCobot esta en
#       ~/jetcobot_colcon_ws/src/jetcobot_description (o en el codigo descargado de Yahboom)
#   importar_modelo_yahboom.sh --descargar
#       descarga la copia del paquete de Yahboom publicada en GitHub por un tercero
#       (DiegoRobotics/two-arm-insert-station, commit f186491)
set -euo pipefail

WS="${ROBOTS_WS:-$HOME/robots_ws}"
DEST="$WS/src/jc_description/meshes/yahboom"
FILES="base_link.STL 1_Link.STL 2_Link.STL 3_Link.STL 4_Link.STL 5_Link.STL 6_Link.STL jiazhua_Link.STL camera_Link.STL"
URL="https://raw.githubusercontent.com/DiegoRobotics/two-arm-insert-station/f1864919a336e7890c6255acbb6468dedba5fae1/JetCobot-Placer/ros_ws/src/jetcobot_description/meshes"

if [ "$#" -ne 1 ]; then
    sed -n '2,15p' "$0"; exit 1
fi
mkdir -p "$DEST"
if [ "$1" = "--descargar" ]; then
    echo "Descargando las mallas de Yahboom (copia publicada por un tercero) ..."
    for f in $FILES; do
        curl -fsSL -o "$DEST/$f" "$URL/$f"
        echo "  $f"
    done
else
    SRC="$1/meshes"
    [ -d "$SRC" ] || SRC="$1"
    for f in $FILES; do
        [ -f "$SRC/$f" ] || { echo "No se encontro $SRC/$f"; exit 1; }
        cp "$SRC/$f" "$DEST/$f"
        echo "  $f"
    done
fi
for f in $FILES; do
    [ "$(stat -c %s "$DEST/$f")" -gt 10000 ] || { echo "Archivo incompleto: $DEST/$f"; exit 1; }
done
echo "Mallas instaladas en $DEST"
echo "Compile:  cd $WS && colcon build --symlink-install --packages-select jc_description"
