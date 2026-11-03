#!/bin/bash
# Inicia el contenedor ROS 2 Humble en la Jetson Nano con acceso a los puertos del robot.
#   ./run_nano.sh                      -> terminal interactiva
#   ./run_nano.sh ros2 launch mr_bringup robot.launch.py camera:=true
# --network host / --ipc host: DDS descubre a los demas equipos (PC, AGX) como si corriera en la Nano.
# --privileged + /dev: RoboClaw (/dev/roboclaw), RPLidar (/dev/rplidar) y camara (/dev/video*).
WS=${ROBOTS_WS:-$HOME/robots_ws}
# Sin argumentos abre una terminal; con argumentos ejecuta el comando con ROS 2, mr_deps y robots_ws cargados.
if [ $# -eq 0 ]; then
  CMD=(bash)
else
  CMD=(bash -c "source /opt/ros/humble/install/setup.bash; source /opt/mr_deps/install/setup.bash; \
[ -f /root/robots_ws/install/setup.bash ] && source /root/robots_ws/install/setup.bash; exec $*")
fi
docker run -it --rm --runtime nvidia --network host --ipc host --privileged \
  -v /dev:/dev -v "$WS":/root/robots_ws \
  -e ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-20} \
  --name mr_robot mr_robot:humble-nano "${CMD[@]}"
