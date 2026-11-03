"""Ordenes de navegacion desde la terminal.

  ros2 run mr_navigation ir_a estacion_A          # va al punto y espera el resultado
  ros2 run mr_navigation recorrido cobot estacion_A estacion_B inicio [--vueltas 2]
  ros2 run mr_navigation guardar_punto estacion_D  # guarda la pose actual con ese nombre
  ros2 run mr_navigation estaciones               # lista los puntos definidos
"""
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import Trigger

from mr_interfaces.action import GoToStation
from mr_interfaces.srv import SavePoint


def _go(node, client, name):
    if not client.wait_for_server(timeout_sec=10.0):
        print('station_manager no responde: lance mr_bringup navigation.launch.py')
        return False

    def fb(m):
        print(f'\r  {name}: faltan {m.feedback.distance_remaining:5.2f} m', end='', flush=True)

    f = client.send_goal_async(GoToStation.Goal(name=name), feedback_callback=fb)
    rclpy.spin_until_future_complete(node, f)
    gh = f.result()
    if not gh.accepted:
        print(f'Meta rechazada: "{name}" no existe (use: ros2 run mr_navigation estaciones)')
        return False
    r = gh.get_result_async()
    try:
        rclpy.spin_until_future_complete(node, r)
    except KeyboardInterrupt:
        c = gh.cancel_goal_async()
        rclpy.spin_until_future_complete(node, c)
        print('\nCancelado')
        return False
    res = r.result().result
    print(f'\n  {res.message} | error final {res.final_distance * 100:.1f} cm')
    return res.success


def ir_a():
    rclpy.init(args=sys.argv)
    args = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    if not args:
        print('Uso: ros2 run mr_navigation ir_a <estacion>')
        return
    node = Node('ir_a')
    ok = _go(node, ActionClient(node, GoToStation, '/mr/go_to_station'), args[0])
    rclpy.try_shutdown()
    sys.exit(0 if ok else 1)


def recorrido():
    rclpy.init(args=sys.argv)
    args = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    vueltas = 1
    if '--vueltas' in args:
        i = args.index('--vueltas')
        vueltas = int(args[i + 1])
        del args[i:i + 2]
    node = Node('recorrido')
    cli = ActionClient(node, GoToStation, '/mr/go_to_station')
    resumen = []
    for v in range(vueltas):
        for name in args:
            print(f'[vuelta {v + 1}/{vueltas}] -> {name}')
            resumen.append((name, _go(node, cli, name)))
    print('Resumen:', ', '.join(f'{n}={"OK" if ok else "FALLO"}' for n, ok in resumen))
    rclpy.try_shutdown()


def _call(srv_type, name, req):
    node = Node('mr_cli')
    cli = node.create_client(srv_type, name)
    if not cli.wait_for_service(timeout_sec=5.0):
        print(f'Servicio {name} no disponible')
        return None
    f = cli.call_async(req)
    rclpy.spin_until_future_complete(node, f)
    return f.result()


def guardar_punto():
    rclpy.init(args=sys.argv)
    args = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    if not args:
        print('Uso: ros2 run mr_navigation guardar_punto <nombre>')
        return
    r = _call(SavePoint, '/mr/save_point', SavePoint.Request(name=args[0]))
    if r:
        print(r.message)
    rclpy.try_shutdown()


def estaciones():
    rclpy.init(args=sys.argv)
    r = _call(Trigger, '/mr/list_points', Trigger.Request())
    if r:
        print(r.message.replace('; ', '\n'))
    rclpy.try_shutdown()
