"""Cliente sincronico de los servicios de jc_motion/motion_server (uso desde Python)."""
import math
import threading

from geometry_msgs.msg import PoseStamped

from jc_interfaces.srv import MoveToNamed, MoveToPose, SetGripper


def top_down_quat(yaw):
    """Cuaternion (x,y,z,w) del TCP apuntando hacia abajo: roll = pi, pitch = 0, yaw dado."""
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (cy, sy, 0.0, 0.0)  # = quat_from_rpy(pi, 0, yaw)


class MotionClient:
    def __init__(self, node, timeout=60.0):
        self.node, self.timeout = node, timeout
        self.c_pose = node.create_client(MoveToPose, "/jc/move_to_pose")
        self.c_named = node.create_client(MoveToNamed, "/jc/move_to_named")
        self.c_grip = node.create_client(SetGripper, "/jc/set_gripper")

    def wait(self, timeout=30.0):
        return all(c.wait_for_service(timeout_sec=timeout) for c in (self.c_pose, self.c_named, self.c_grip))

    def _call(self, client, req):
        ev = threading.Event()
        fut = client.call_async(req)
        fut.add_done_callback(lambda _f: ev.set())
        if not ev.wait(self.timeout):
            return False, "timeout"
        r = fut.result()
        return r.success, r.message

    def named(self, name, vel=0.0):
        return self._call(self.c_named, MoveToNamed.Request(name=name, velocity_scaling=vel))

    def gripper(self, opening):
        return self._call(self.c_grip, SetGripper.Request(opening=float(opening)))

    def pose(self, x, y, z, yaw=-math.pi / 2, frame="world", cartesian=False, vel=0.0):
        ps = PoseStamped()
        ps.header.frame_id = frame
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = float(x), float(y), float(z)
        q = top_down_quat(yaw)
        ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = q
        return self._call(self.c_pose, MoveToPose.Request(target=ps, cartesian=cartesian, velocity_scaling=vel))
