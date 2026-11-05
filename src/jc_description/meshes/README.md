# Mallas del JetCobot de Yahboom

`yahboom/` guarda las mallas STL del modelo de Yahboom (base, eslabones 1 a 6, pinza y cámara).
No se publican en este repositorio: Yahboom no les declaró licencia. Se instalan en cada equipo con

    ~/robots_ws/src/jc_description/scripts/importar_modelo_yahboom.sh --descargar

o, si tiene la imagen de fábrica del JetCobot, con la carpeta `~/jetcobot_colcon_ws/src/jetcobot_description`
en lugar de `--descargar`. Después: `colcon build --symlink-install --packages-select jc_description`.

Sin estas mallas, `display.launch.py` y los launch de la celda dibujan el modelo simple (primitivas),
con la misma cinemática.
