"""Crear pedidos desde la terminal (alternativa a la App web).

  ros2 run mr_logistics pedido rojo 2 estacion_A
"""
import sys

import rclpy
from rclpy.node import Node

from mr_interfaces.msg import Order
from mr_interfaces.srv import SubmitOrder


def main():
    rclpy.init(args=sys.argv)
    a = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    if len(a) != 3:
        print('Uso: ros2 run mr_logistics pedido <producto> <cantidad> <estacion_destino>')
        return
    n = Node('pedido_cli')
    c = n.create_client(SubmitOrder, '/logistics/submit_order')
    if not c.wait_for_service(timeout_sec=5.0):
        print('mission_manager no disponible')
        return
    f = c.call_async(SubmitOrder.Request(order=Order(product=a[0], quantity=int(a[1]), destination=a[2])))
    rclpy.spin_until_future_complete(n, f)
    r = f.result()
    print(('ACEPTADO ' if r.accepted else 'RECHAZADO ') + r.message)
    rclpy.try_shutdown()
