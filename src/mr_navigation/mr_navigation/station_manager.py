"""Gestor de estaciones: puntos con nombre en el mapa y orden "ir al punto X".

Servicios
  /mr/go_to        (mr_interfaces/GoTo)        envia el robot a una estacion (no bloqueante)
  /mr/save_point   (mr_interfaces/SavePoint)   guarda la pose actual (TF map->base_footprint) con un nombre
  /mr/list_points  (std_srvs/Trigger)          lista las estaciones en el campo 'message'
  /mr/cancel       (std_srvs/Trigger)          cancela la navegacion en curso
Accion
  /mr/go_to_station (mr_interfaces/GoToStation)  igual que go_to pero con realimentacion y resultado
Topicos
  /mr/stations_markers (visualization_msgs/MarkerArray, latched)  flechas y nombres en RViz
  /mr/nav_state        (std_msgs/String)                          estado legible para la App web
Internamente usa la accion de Nav2 'navigate_to_pose' (nav2_msgs/NavigateToPose).
"""
import math
import threading

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from mr_interfaces.action import GoToStation
from mr_interfaces.srv import GoTo, SavePoint

from . import stations as st

STATUS_TXT = {GoalStatus.STATUS_SUCCEEDED: 'EXITO', GoalStatus.STATUS_ABORTED: 'ABORTADO',
              GoalStatus.STATUS_CANCELED: 'CANCELADO'}


class StationManager(Node):
    def __init__(self):
        super().__init__('station_manager')
        self.file = self.declare_parameter('stations_file', 'estaciones.yaml').value
        self.base_frame = self.declare_parameter('base_frame', 'base_footprint').value
        self.frame, self.stations = st.load(self.file)
        self.cb = ReentrantCallbackGroup()
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose', callback_group=self.cb)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.lock = threading.Lock()
        self.active = None            # (nombre, goal_handle de nav2)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.markers_pub = self.create_publisher(MarkerArray, '/mr/stations_markers', latched)
        self.state_pub = self.create_publisher(String, '/mr/nav_state', 10)
        self.create_service(GoTo, '/mr/go_to', self.on_go_to, callback_group=self.cb)
        self.create_service(SavePoint, '/mr/save_point', self.on_save, callback_group=self.cb)
        self.create_service(Trigger, '/mr/list_points', self.on_list, callback_group=self.cb)
        self.create_service(Trigger, '/mr/cancel', self.on_cancel, callback_group=self.cb)
        ActionServer(self, GoToStation, '/mr/go_to_station', self.execute, callback_group=self.cb,
                     goal_callback=self.on_goal, cancel_callback=lambda _g: CancelResponse.ACCEPT)
        self.publish_markers()
        self.get_logger().info(f'{len(self.stations)} estaciones cargadas de {self.file}: '
                               f'{", ".join(self.stations) or "(ninguna)"}')

    # ------------------------------------------------------------------ utilidades
    def say(self, text):
        self.state_pub.publish(String(data=text))
        self.get_logger().info(text)

    def pose_of(self, s: st.Station) -> PoseStamped:
        p = PoseStamped()
        p.header.frame_id = self.frame
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x, p.pose.position.y = s.x, s.y
        p.pose.orientation.z, p.pose.orientation.w = math.sin(s.yaw / 2), math.cos(s.yaw / 2)
        return p

    def robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(self.frame, self.base_frame, rclpy.time.Time(),
                                                timeout=Duration(seconds=0.5))
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'Sin TF {self.frame}->{self.base_frame}: {e}')
            return None
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return t.transform.translation.x, t.transform.translation.y, yaw

    def publish_markers(self):
        arr = MarkerArray()
        clear = Marker(action=Marker.DELETEALL)
        clear.header.frame_id = self.frame
        arr.markers.append(clear)
        for i, s in enumerate(self.stations.values()):
            arrow = Marker(ns='estaciones', id=2 * i, type=Marker.ARROW, action=Marker.ADD)
            arrow.header.frame_id = self.frame
            arrow.pose = self.pose_of(s).pose
            arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.35, 0.06, 0.06
            arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = 0.22, 0.66, 0.0, 1.0  # verde SENA
            text = Marker(ns='nombres', id=2 * i + 1, type=Marker.TEXT_VIEW_FACING, action=Marker.ADD, text=s.name)
            text.header.frame_id = self.frame
            text.pose.position.x, text.pose.position.y, text.pose.position.z = s.x, s.y, 0.35
            text.scale.z = 0.22
            text.color.r = text.color.g = text.color.b = text.color.a = 1.0
            arr.markers += [arrow, text]
        self.markers_pub.publish(arr)

    # ------------------------------------------------------------------ servicios
    def on_list(self, _req, res):
        res.success = True
        res.message = '; '.join(f'{s.name}: ({s.x:.2f}, {s.y:.2f}, {math.degrees(s.yaw):.0f} grados)'
                                for s in self.stations.values()) or '(sin estaciones)'
        return res

    def on_save(self, req, res):
        if not st.valid_name(req.name):
            res.success, res.message = False, 'Nombre invalido (letras, numeros, _ o -; maximo 40)'
            return res
        pose = self.robot_pose()
        if pose is None:
            res.success, res.message = False, 'No hay localizacion (TF map->base_footprint)'
            return res
        with self.lock:
            self.stations[req.name] = st.Station(req.name, *pose)
            st.save(self.file, self.frame, self.stations)
        self.publish_markers()
        res.success = True
        res.message = (f'{req.name} = ({pose[0]:.3f}, {pose[1]:.3f}, {math.degrees(pose[2]):.1f} grados) '
                       f'guardado en {self.file}')
        self.get_logger().info(res.message)
        return res

    def on_go_to(self, req, res):
        if req.name not in self.stations:
            res.accepted, res.message = False, f'Estacion desconocida: {req.name}. Opciones: {", ".join(self.stations)}'
            return res
        threading.Thread(target=self.navigate, args=(req.name, None), daemon=True).start()
        res.accepted, res.message = True, f'Navegando a {req.name}'
        return res

    def on_cancel(self, _req, res):
        with self.lock:
            active = self.active
        if active and active[1] is not None:
            active[1].cancel_goal_async()
            res.success, res.message = True, f'Cancelando {active[0]}'
        else:
            res.success, res.message = False, 'No hay navegacion activa'
        return res

    # ------------------------------------------------------------------ accion
    def on_goal(self, goal):
        return GoalResponse.ACCEPT if goal.name in self.stations else GoalResponse.REJECT

    def execute(self, gh):
        ok, msg, dist = self.navigate(gh.request.name, gh)
        result = GoToStation.Result(success=ok, message=msg, final_distance=float(dist))
        if gh.is_cancel_requested:
            gh.canceled()
        elif ok:
            gh.succeed()
        else:
            gh.abort()
        return result

    def navigate(self, name, gh=None):
        s = self.stations[name]
        if not self.nav.wait_for_server(timeout_sec=10.0):
            self.say('Nav2 no disponible (navigate_to_pose)')
            return False, 'Nav2 no disponible', -1.0
        with self.lock:
            if self.active and self.active[1] is not None:
                self.active[1].cancel_goal_async()        # una sola meta a la vez: la nueva reemplaza
            self.active = (name, None)
        goal = NavigateToPose.Goal(pose=self.pose_of(s))
        done = threading.Event()
        state = {}

        def feedback(fb):
            d = fb.feedback.distance_remaining
            if gh is not None:
                gh.publish_feedback(GoToStation.Feedback(distance_remaining=float(d), nav_status='NAVEGANDO'))

        send = self.nav.send_goal_async(goal, feedback_callback=feedback)
        send.add_done_callback(lambda f: (state.__setitem__('gh', f.result()), done.set()))
        done.wait(10.0)
        nav_gh = state.get('gh')
        if nav_gh is None or not nav_gh.accepted:
            self.say(f'Nav2 rechazo la meta {name}')
            return False, 'Meta rechazada por Nav2', -1.0
        with self.lock:
            self.active = (name, nav_gh)
        self.say(f'NAVEGANDO hacia {name}')
        done.clear()
        res_f = nav_gh.get_result_async()
        res_f.add_done_callback(lambda f: done.set())
        while not done.wait(0.2):
            if gh is not None and gh.is_cancel_requested:
                nav_gh.cancel_goal_async()
        status = res_f.result().status
        with self.lock:
            if self.active and self.active[1] is nav_gh:
                self.active = None
        pose = self.robot_pose()
        dist = math.hypot(pose[0] - s.x, pose[1] - s.y) if pose else -1.0
        txt = STATUS_TXT.get(status, f'ESTADO_{status}')
        self.say(f'{txt}: {name} (error final {dist * 100:.1f} cm)')
        return status == GoalStatus.STATUS_SUCCEEDED, f'{txt} en {name}', dist


def main():
    rclpy.init()
    node = StationManager()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
