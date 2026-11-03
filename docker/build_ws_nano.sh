#!/bin/bash
# Compila dentro del contenedor los paquetes que ya se incorporaron a ~/robots_ws/src (se agregan paso a paso:
# mr_base en el DA-11.2, mr_description y mr_bringup en el DA-12.1, mr_perception en el DA-15.1).
#   build_ws_nano.sh                 -> compila todo lo que hay en src/
#   build_ws_nano.sh mr_bringup ...  -> compila solo esos paquetes
# SLAM, Nav2 y la App corren en el PC o en la AGX Orin. No se usa rosdep (instalaria paquetes apt de 18.04).
cd /root/robots_ws && source /opt/ros/humble/install/setup.bash && source /opt/mr_deps/install/setup.bash
if [ $# -eq 0 ]; then
  colcon build --symlink-install
else
  colcon build --symlink-install --packages-select "$@"
fi
