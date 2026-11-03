"""Publica detecciones sinteticas (prueba de pick & place sin camara)."""
import sys
import rclpy
from rclpy.node import Node
from jc_interfaces.msg import DetectedObject, DetectedObjectArray


def main():
    rclpy.init()
    n = Node("fake_detections", parameter_overrides=[])
    x, y, z = map(float, sys.argv[1:4]) if len(sys.argv) > 3 else (0.16, -0.06, 0.025)
    pub = n.create_publisher(DetectedObjectArray, "/jc/detections", 10)

    def tick():
        m = DetectedObjectArray()
        m.header.stamp = n.get_clock().now().to_msg()
        m.header.frame_id = "world"
        o = DetectedObject(label="rojo", shape="cuadrado", score=1.0, pose_valid=True)
        o.header = m.header
        o.pose.position.x, o.pose.position.y, o.pose.position.z = x, y, z
        o.pose.orientation.w = 1.0
        m.objects.append(o)
        pub.publish(m)
    n.create_timer(0.1, tick)
    rclpy.spin(n)


if __name__ == "__main__":
    main()
